#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


class PointCloudSelfFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_self_filter")

        self.declare_parameter("input_topic", "/unitree4d_l2_lidar")
        self.declare_parameter("output_topic", "/unitree4d_l2_lidar/filtered")
        self.declare_parameter("filter_frame", "base_footprint")
        self.declare_parameter("min_x", -0.45)
        self.declare_parameter("max_x", 0.45)
        self.declare_parameter("min_y", -0.45)
        self.declare_parameter("max_y", 0.45)
        self.declare_parameter("min_z", 0.0)
        self.declare_parameter("max_z", 1.9)
        self.declare_parameter("min_range", 0.75)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.filter_frame = self.get_parameter("filter_frame").value
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.min_range = float(self.get_parameter("min_range").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Filtering {self.input_topic} -> {self.output_topic} in {self.filter_frame}"
        )

    def cloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.filter_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        points = point_cloud2.read_points(msg, skip_nans=True)
        if len(points) == 0:
            self.publisher.publish(msg)
            return

        filtered_xyz = self.filter_points(points, transform.transform)
        output = point_cloud2.create_cloud_xyz32(msg.header, filtered_xyz)
        output.is_dense = True
        self.publisher.publish(output)

    def filter_points(self, points, transform):
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32)
        finite_mask = np.isfinite(xyz).all(axis=1)

        keep_mask = finite_mask
        if self.min_range > 0.0:
            keep_mask &= np.einsum("ij,ij->i", xyz, xyz) >= self.min_range * self.min_range

        rotation = quaternion_to_matrix(transform.rotation)
        translation = np.array(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ]
        )
        transform_xyz = np.nan_to_num(xyz, nan=0.0, posinf=0.0, neginf=0.0)
        points_in_filter_frame = transform_xyz @ rotation.T + translation

        inside_robot = (
            (points_in_filter_frame[:, 0] >= self.min_x)
            & (points_in_filter_frame[:, 0] <= self.max_x)
            & (points_in_filter_frame[:, 1] >= self.min_y)
            & (points_in_filter_frame[:, 1] <= self.max_y)
            & (points_in_filter_frame[:, 2] >= self.min_z)
            & (points_in_filter_frame[:, 2] <= self.max_z)
        )
        keep_mask &= ~inside_robot

        return xyz[keep_mask]


def quaternion_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.identity(3)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
