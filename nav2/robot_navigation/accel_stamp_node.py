#!/usr/bin/env python3

import math
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class Accel_Stamp(Node):
    def __init__(self):
        super().__init__("accel_stamp")

        # Parameters
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("accel_linear", 0.5)
        self.declare_parameter("decel_linear", 1.0)
        self.declare_parameter("accel_yaw", 1.0)
        self.declare_parameter("decel_yaw", 1.2)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("max_linear_speed", 1.0)  # for angular scaling
        self.declare_parameter("max_vel",0.6)

        self.frame_id = self.get_parameter("frame_id").value
        self.accel_linear = self.get_parameter("accel_linear").value
        self.decel_linear = self.get_parameter("decel_linear").value
        self.accel_yaw = self.get_parameter("accel_yaw").value
        self.decel_yaw = self.get_parameter("decel_yaw").value
        self.rate = self.get_parameter("rate").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_vel = self.get_parameter("max_vel").value
        self.dt = 1.0 / self.rate

        # State
        self.target = Twist()
        self.current = Twist()

        # Publisher & Subscriber
        self.pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.sub = self.create_subscription(Twist, "cmd_vel_in", self.cb_cmd, 10)
        self.timer = self.create_timer(self.dt, self.smoothing_loop)

    def cb_cmd(self, msg: Twist):
        self.target = msg

    def limit_vector(self, current_vec, target_vec, accel, decel):
        # Compute delta vector
        delta_x = target_vec[0] - current_vec[0]
        delta_y = target_vec[1] - current_vec[1]
        delta_mag = math.hypot(delta_x, delta_y)

        # Choose limit based on speeding up or slowing down
        current_mag = math.hypot(current_vec[0], current_vec[1])
        rate = accel if delta_mag > current_mag else decel
        max_step = rate * self.dt

        # Scale delta vector if it exceeds max_step
        if delta_mag > max_step and delta_mag != 0:
            scale = max_step / delta_mag
            delta_x *= scale
            delta_y *= scale

        return [current_vec[0] + delta_x, current_vec[1] + delta_y]

    def limit_scalar(self, current, target, accel, decel):
        delta = target - current
        rate = accel if abs(target) > abs(current) else decel
        max_step = rate * self.dt
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
        if current + delta >= self.max_vel:
            return self.max_vel
        else:
            return current + delta

    def smoothing_loop(self):
        # --- Linear vector limiting ---
        linear_target = [self.target.linear.x, self.target.linear.y]
        linear_current = [self.current.linear.x, self.current.linear.y]
        linear_next = self.limit_vector(linear_current, linear_target, self.accel_linear, self.decel_linear)
        self.current.linear.x = linear_next[0]
        self.current.linear.y = linear_next[1]

        # Linear Z (optional for omni)
        self.current.linear.z = self.limit_scalar(self.current.linear.z, self.target.linear.z, self.accel_linear, self.decel_linear)

        # --- Angular limiting ---
        # Scale angular.z by linear speed to prevent overshoot
        linear_speed = math.hypot(self.current.linear.x, self.current.linear.y)
        scale = max(0.0, 1.0 - linear_speed / self.max_linear_speed)
        target_angular_z = self.target.angular.z * scale
        self.current.angular.z = self.limit_scalar(self.current.angular.z, target_angular_z, self.accel_yaw, self.decel_yaw)

        # Optional: angular x/y if needed
        self.current.angular.x = self.limit_scalar(self.current.angular.x, self.target.angular.x, self.accel_yaw, self.decel_yaw)
        self.current.angular.y = self.limit_scalar(self.current.angular.y, self.target.angular.y, self.accel_yaw, self.decel_yaw)

        # --- Publish ---
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist = self.current
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Accel_Stamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
