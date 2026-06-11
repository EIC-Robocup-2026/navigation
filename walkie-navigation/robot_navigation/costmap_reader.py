#!/usr/bin/env python3
"""Costmap + robot-pose cache for the nav commander.

Not a Node itself — attaches subscriptions to the node passed in. The table
edge is marked by the ZED head camera into the LOCAL costmap (STVL), which is
a rolling window in the odom frame, while object goals arrive in map. So this
class is frame-aware: it answers "robot pose in frame X" via TF and transforms
points/poses between map and the costmap frame. /amcl_pose is kept as the
fast path for the map-frame robot pose when AMCL is running.
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles
from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped

_TF_ERRORS = (tf2_ros.LookupException,
              tf2_ros.ConnectivityException,
              tf2_ros.ExtrapolationException)


class CostmapReader:
    def __init__(self, node,
                 costmap_topic='/local_costmap/costmap',
                 pose_topic='/amcl_pose',
                 global_frame='map',
                 robot_base_frame='base_footprint'):
        self.node = node
        self.latest_costmap = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self._have_amcl_pose = False
        self.global_frame = global_frame
        self.robot_base_frame = robot_base_frame

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        node.create_subscription(
            OccupancyGrid, costmap_topic, self._costmap_cb, qos)
        node.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._pose_cb, 10)

    def _costmap_cb(self, msg):
        self.latest_costmap = msg

    def _pose_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._have_amcl_pose = True

    def costmap_frame(self):
        if self.latest_costmap is None:
            return None
        return self.latest_costmap.header.frame_id or 'odom'

    def get_robot_pose(self, frame=None) -> tuple[float, float] | None:
        """Robot (x, y) in the given frame (default: global_frame).
        AMCL is the fast path for the map frame; everything else goes
        through TF. Returns None if the pose is unavailable."""
        frame = frame or self.global_frame
        if frame == self.global_frame and self._have_amcl_pose:
            return self.robot_x, self.robot_y
        try:
            t = self._tf_buffer.lookup_transform(
                frame, self.robot_base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except _TF_ERRORS as e:
            self.node.get_logger().warn(
                f'CostmapReader: robot pose in {frame} unavailable: {e}')
            if frame == self.global_frame:
                return self.robot_x, self.robot_y
            return None

    def get_robot_pose_full(self, frame=None) -> tuple[float, float, float] | None:
        """Robot (x, y, yaw) in the given frame (default: global_frame).
        TF first — /amcl_pose only updates every update_min_d of travel, so
        it can be ~0.25 m / stale-heading behind while driving; checks that
        compare against the live robot pose need TF. Falls back to the last
        AMCL pose for the global frame."""
        frame = frame or self.global_frame
        try:
            t = self._tf_buffer.lookup_transform(
                frame, self.robot_base_frame, rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except _TF_ERRORS as e:
            if frame == self.global_frame and self._have_amcl_pose:
                return self.robot_x, self.robot_y, self.robot_yaw
            self.node.get_logger().warn(
                f'CostmapReader: full robot pose in {frame} unavailable: {e}')
            return None

    def transform_point(self, x, y, from_frame, to_frame):
        """(x, y) re-expressed in to_frame, or None if TF fails."""
        if from_frame == to_frame:
            return float(x), float(y)
        ps = PointStamped()
        ps.header.frame_id = from_frame
        ps.point.x = float(x)
        ps.point.y = float(y)
        try:
            t = self._tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            out = do_transform_point(ps, t)
            return out.point.x, out.point.y
        except _TF_ERRORS as e:
            self.node.get_logger().warn(
                f'CostmapReader: point transform {from_frame}->{to_frame} '
                f'failed: {e}')
            return None

    def transform_pose(self, pose_stamped, to_frame):
        """PoseStamped re-expressed in to_frame, or None if TF fails."""
        if pose_stamped.header.frame_id == to_frame:
            return pose_stamped
        try:
            t = self._tf_buffer.lookup_transform(
                to_frame, pose_stamped.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            return do_transform_pose_stamped(pose_stamped, t)
        except _TF_ERRORS as e:
            self.node.get_logger().warn(
                f'CostmapReader: pose transform '
                f'{pose_stamped.header.frame_id}->{to_frame} failed: {e}')
            return None

    def is_ready(self) -> bool:
        return self.latest_costmap is not None
