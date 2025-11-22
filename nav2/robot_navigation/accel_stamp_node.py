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
        self.declare_parameter("decel_linear", 1.5)
        self.declare_parameter("accel_yaw", 1.0)
        self.declare_parameter("decel_yaw", 1.5)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("max_linear_speed", 1.0)  # for angular scaling
        
        # NEW SEPARATED LIMITS:
        self.declare_parameter("max_linear_vel_final", 0.6)  # Max magnitude for linear.x/y/z
        self.declare_parameter("max_angular_vel_final", 1.0) # Max magnitude for angular.z/x/y

        self.frame_id = self.get_parameter("frame_id").value
        self.accel_linear = self.get_parameter("accel_linear").value
        self.decel_linear = self.get_parameter("decel_linear").value
        self.accel_yaw = self.get_parameter("accel_yaw").value
        self.decel_yaw = self.get_parameter("decel_yaw").value
        self.rate = self.get_parameter("rate").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        
        # Retrieve NEW SEPARATED LIMITS
        self.max_linear_vel_final = self.get_parameter("max_linear_vel_final").value
        self.max_angular_vel_final = self.get_parameter("max_angular_vel_final").value
        
        self.dt = 1.0 / self.rate

        # State
        self.target = Twist()
        self.current = Twist()

        # Publisher & Subscriber
        self.pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.sub = self.create_subscription(Twist, "cmd_vel_in", self.cb_cmd, 10)
        self.timer = self.create_timer(self.dt, self.smoothing_loop)

    def cb_cmd(self, msg: Twist):
        # 1. Linear Target Cap (using max_linear_vel_final)
        target_mag = math.hypot(msg.linear.x, msg.linear.y)
        if target_mag > self.max_linear_vel_final:
            scale = self.max_linear_vel_final / target_mag
            msg.linear.x *= scale
            msg.linear.y *= scale

        # 2. Scalar Target Caps (using max_linear_vel_final and max_angular_vel_final)
        msg.linear.z = max(-self.max_linear_vel_final, min(self.max_linear_vel_final, msg.linear.z))
        
        # Apply the ANGULAR limit for angular components
        msg.angular.x = max(-self.max_angular_vel_final, min(self.max_angular_vel_final, msg.angular.x))
        msg.angular.y = max(-self.max_angular_vel_final, min(self.max_angular_vel_final, msg.angular.y))
        msg.angular.z = max(-self.max_angular_vel_final, min(self.max_angular_vel_final, msg.angular.z))
        
        self.target = msg

    def limit_vector(self, current_vec, target_vec, accel, decel):
        # Compute delta vector
        delta_x = target_vec[0] - current_vec[0]
        delta_y = target_vec[1] - current_vec[1]
        delta_mag = math.hypot(delta_x, delta_y)

        # Choose limit based on speeding up or slowing down
        current_mag = math.hypot(current_vec[0], current_vec[1])
        rate = accel if abs(delta_mag) > 0.001 and math.copysign(1.0, delta_x * current_vec[0] + delta_y * current_vec[1]) > 0 else decel
        max_step = rate * self.dt

        # Scale delta vector if it exceeds max_step
        if delta_mag > max_step and delta_mag != 0:
            scale = max_step / delta_mag
            delta_x *= scale
            delta_y *= scale

        next_x = current_vec[0] + delta_x
        next_y = current_vec[1] + delta_y

        # Global Max LINEAR Velocity Cap
        next_mag = math.hypot(next_x, next_y)
        if next_mag > self.max_linear_vel_final and self.max_linear_vel_final > 0:
            scale_cap = self.max_linear_vel_final / next_mag
            next_x *= scale_cap
            next_y *= scale_cap

        return [next_x, next_y]

    def limit_scalar(self, current, target, accel, decel, max_limit):
        # Target must be capped at max_limit, which is now handled in cb_cmd
        
        delta = target - current
        
        # Choose rate: speeding up or slowing down
        rate = accel if abs(target) > abs(current) else decel
        max_step = rate * self.dt
        
        # Limit the delta
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
            
        next_vel = current + delta
        
        # Global Max Velocity Cap (using the provided max_limit)
        next_vel = max(-max_limit, min(max_limit, next_vel))

        return next_vel

    def smoothing_loop(self):
        # --- Linear vector limiting (uses max_linear_vel_final inside limit_vector) ---
        linear_target = [self.target.linear.x, self.target.linear.y]
        linear_current = [self.current.linear.x, self.current.linear.y]
        linear_next = self.limit_vector(linear_current, linear_target, self.accel_linear, self.decel_linear)
        self.current.linear.x = linear_next[0]
        self.current.linear.y = linear_next[1]

        # Linear Z (optional for omni) - Uses max_linear_vel_final
        self.current.linear.z = self.limit_scalar(self.current.linear.z, self.target.linear.z, self.accel_linear, self.decel_linear, self.max_linear_vel_final)

        # --- Angular limiting ---
        # Scale angular.z by linear speed to prevent overshoot
        linear_speed = math.hypot(self.current.linear.x, self.current.linear.y)
        scale = max(0.0, 1.0 - linear_speed / self.max_linear_speed)
        target_angular_z = self.target.angular.z * scale
        
        # Angular Z - Uses max_angular_vel_final
        self.current.angular.z = self.limit_scalar(self.current.angular.z, target_angular_z, self.accel_yaw, self.decel_yaw, self.max_angular_vel_final)

        # Optional: angular x/y if needed - Uses max_angular_vel_final
        self.current.angular.x = self.limit_scalar(self.current.angular.x, self.target.angular.x, self.accel_yaw, self.decel_yaw, self.max_angular_vel_final)
        self.current.angular.y = self.limit_scalar(self.current.angular.y, self.target.angular.y, self.accel_yaw, self.decel_yaw, self.max_angular_vel_final)

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