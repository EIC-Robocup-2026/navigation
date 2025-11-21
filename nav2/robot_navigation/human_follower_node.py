#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
import math

SAFE_DISTANCE = 1.2  # meters away from human

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')
        
        # Publisher for Nav2 goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.previous_goal_x = None
        self.previous_goal_y = None

        # Timer to update goal regularly
        self.timer = self.create_timer(0.5, self.update_goal)  # 2 Hz

    def update_goal(self):
        try:
            # Lookup transform from map to detected_human
            t = self.tf_buffer.lookup_transform('map', 'detected_human', rclpy.time.Time())

            if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
                self.get_logger().warn("Human TF timestamp is zero — probably lost.")
                return

            # --- CHECK FOR STALE TF DATA ---
            now_ns = self.get_clock().now().nanoseconds
            tf_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            age = (now_ns - tf_ns) / 1e9

            if age > 0.5:  # human not seen recently
                self.get_logger().warn(f"Human TF too old ({age:.2f}s). Not sending goal.")
                return
            
            human_x = t.transform.translation.x
            human_y = t.transform.translation.y
            
            # Get human yaw
            q = t.transform.rotation
            _, _, human_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Goal offset to maintain safe distance
            goal_x = human_x - SAFE_DISTANCE * math.cos(human_yaw)
            goal_y = human_y - SAFE_DISTANCE * math.sin(human_yaw)
            
            # Create PoseStamped goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0.0
            
            # Orient the robot facing the human
            quat = tf_transformations.quaternion_from_euler(0, 0, human_yaw)
            goal.pose.orientation.x = quat[0]
            goal.pose.orientation.y = quat[1]
            goal.pose.orientation.z = quat[2]
            goal.pose.orientation.w = quat[3]

            if self.previous_goal_x is None or self.previous_goal_y is None:
                self.goal_pub.publish(goal)
                self.get_logger().info(f'Published FIRST goal at ({goal_x:.2f}, {goal_y:.2f})')
                self.previous_goal_x = goal_x
                self.previous_goal_y = goal_y
                return
            
            distance_to_goal = math.hypot(goal_x - self.previous_goal_x, goal_y - self.previous_goal_y)

            if distance_to_goal > 0.05:
                self.goal_pub.publish(goal)
                self.get_logger().info(f'Published goal at ({goal_x:.2f}, {goal_y:.2f})')
                self.previous_goal_x = goal_x
                self.previous_goal_y = goal_y
                return


        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TransformException):
            self.get_logger().warn('Human TF not available. Not sending goal.')
            return


def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
