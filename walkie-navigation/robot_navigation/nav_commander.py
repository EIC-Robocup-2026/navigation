#!/usr/bin/env python3
"""NavCommander: action server that turns raw object positions into Nav2 goals.

The table edge is only marked into the LOCAL costmap (STVL from the ZED head
camera), and only once the camera actually sees the table. So navigation is
two-phase:

  1. face_target — walk to a standoff point on the robot->object line, facing
     the object, so the camera brings the table edge into the costmap.
  2. nearest_edge — re-run the PCA edge fit on the now-populated local
     costmap and refine position + heading to be perpendicular to the edge,
     directly in front of the object.

Phase 1 is skipped when the edge is already visible at goal time. The local
costmap lives in the odom frame, so the PCA runs in odom and the result is
transformed back to map before being sent to Nav2.

RViz integration: the "Publish Point" tool (/clicked_point) places an object
and triggers the full pipeline; debug markers visualize every algorithm stage
and can be toggled at runtime with `ros2 param set /nav_commander debug true`.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from robot_navigation.action import NavigateToObject

# Installed as flat scripts in lib/robot_navigation, imported as siblings
from approach_pose import ApproachPoseComputer
from costmap_reader import CostmapReader


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')

        self.declare_parameter('standoff', 0.65)
        self.declare_parameter('inflation_radius', 0.30)
        self.declare_parameter('search_radius', 1.20)
        self.declare_parameter('min_occupied_cells', 5)
        self.declare_parameter('front_edge_tolerance', 0.25)
        self.declare_parameter('default_method', 'nearest_edge')
        self.declare_parameter('fallback_method', 'face_target')
        self.declare_parameter('two_phase', True)
        self.declare_parameter('refine_position_tolerance', 0.08)
        self.declare_parameter('refine_yaw_tolerance_deg', 8.0)
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('robot_pose_topic', '/amcl_pose')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('navigate_to_object_action', 'navigate_to_object')
        self.declare_parameter('approach_pose_topic', '~/approach_pose')
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_markers_topic', '~/debug_markers')

        config = {
            'standoff': self.get_parameter('standoff').value,
            'inflation_radius': self.get_parameter('inflation_radius').value,
            'search_radius': self.get_parameter('search_radius').value,
            'min_occupied_cells': self.get_parameter('min_occupied_cells').value,
            'front_edge_tolerance': self.get_parameter('front_edge_tolerance').value,
        }
        self._default_method = self.get_parameter('default_method').value
        self._two_phase = self.get_parameter('two_phase').value
        self._refine_pos_tol = self.get_parameter('refine_position_tolerance').value
        self._refine_yaw_tol = math.radians(
            self.get_parameter('refine_yaw_tolerance_deg').value)
        self._global_frame = self.get_parameter('global_frame').value

        self._costmap_reader = CostmapReader(
            self,
            costmap_topic=self.get_parameter('costmap_topic').value,
            pose_topic=self.get_parameter('robot_pose_topic').value,
            global_frame=self._global_frame,
            robot_base_frame=self.get_parameter('robot_base_frame').value,
        )
        self._computer = ApproachPoseComputer(config, logger=self.get_logger())

        # Debug/RViz: latched copy of the last computed approach pose
        self._approach_pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('approach_pose_topic').value, 10)

        # Debug markers: occupied ROI cells, front-edge strip, PCA
        # axis/normal, object projection and approach arrow, plus a boxed text
        # panel. The publisher always exists; the `debug` flag gates publishing
        # and is runtime-settable (ros2 param set /nav_commander debug true).
        self._debug = self.get_parameter('debug').value
        self._debug_pub = self.create_publisher(
            MarkerArray, self.get_parameter('debug_markers_topic').value, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        if self._debug:
            self.get_logger().info('Debug markers enabled')

        self._cb_group = ReentrantCallbackGroup()
        action_name = self.get_parameter('navigate_to_object_action').value
        self._nav_client = ActionClient(
            self, NavigateToPose,
            self.get_parameter('navigate_to_pose_action').value,
            callback_group=self._cb_group)
        self._action_server = ActionServer(
            self, NavigateToObject, action_name,
            execute_callback=self._execute_cb,
            callback_group=self._cb_group)

        # RViz "Publish Point" tool -> treat the click as an object position
        # and run the full pipeline through our own action server.
        self._self_client = ActionClient(
            self, NavigateToObject, action_name,
            callback_group=self._cb_group)
        self.create_subscription(
            PointStamped, self.get_parameter('clicked_point_topic').value,
            self._clicked_point_cb, 10)

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(
                'Nav2 navigate_to_pose action server not available after 10s; '
                'goals will fail until Nav2 is up')

        self.get_logger().info('NavCommander ready: /navigate_to_object')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'debug':
                self._debug = bool(p.value)
                self.get_logger().info(
                    f"Debug markers {'enabled' if self._debug else 'disabled'}")
        return SetParametersResult(successful=True)

    def _clicked_point_cb(self, msg):
        x, y = msg.point.x, msg.point.y
        if msg.header.frame_id and msg.header.frame_id != self._global_frame:
            pt = self._costmap_reader.transform_point(
                x, y, msg.header.frame_id, self._global_frame)
            if pt is None:
                self.get_logger().warn(
                    f'Clicked point in {msg.header.frame_id} could not be '
                    f'transformed to {self._global_frame}, ignoring')
                return
            x, y = pt
        self.get_logger().info(
            f'RViz clicked object at ({x:.3f}, {y:.3f}), sending goal')
        goal = NavigateToObject.Goal()
        goal.obj_x = float(x)
        goal.obj_y = float(y)
        self._self_client.send_goal_async(goal)

    # ------------------------------------------------------------------
    # Goal execution
    # ------------------------------------------------------------------

    def _try_nearest_edge(self, obj_x, obj_y, standoff):
        """Run the PCA edge fit in the costmap frame (odom for the rolling
        local costmap). Returns the approach pose transformed to the global
        frame, or None when the edge is not (yet) visible."""
        costmap = self._costmap_reader.latest_costmap
        if costmap is None:
            self.get_logger().warn('No costmap received yet')
            return None
        frame = self._costmap_reader.costmap_frame()
        obj = self._costmap_reader.transform_point(
            obj_x, obj_y, self._global_frame, frame)
        robot = self._costmap_reader.get_robot_pose(frame)
        if obj is None or robot is None:
            return None
        pose = self._computer.compute(
            obj[0], obj[1], robot[0], robot[1], 'nearest_edge', costmap,
            standoff=standoff, allow_fallback=False)
        self._publish_debug_markers(frame)
        if pose is None:
            return None
        pose.header.frame_id = frame
        return self._costmap_reader.transform_pose(pose, self._global_frame)

    def _face_target_pose(self, obj_x, obj_y, standoff):
        robot = self._costmap_reader.get_robot_pose(self._global_frame)
        if robot is None:
            return None
        pose = self._computer.compute(
            obj_x, obj_y, robot[0], robot[1], 'face_target', None,
            standoff=standoff)
        self._publish_debug_markers(self._global_frame)
        return pose

    async def _navigate(self, goal_handle, pose):
        """Send one pose to Nav2 and await the outcome. Returns (ok, msg)."""
        yaw = _yaw_from_quaternion(pose.pose.orientation)
        self.get_logger().info(
            f'Navigating to x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, yaw={math.degrees(yaw):.1f} deg')

        if not self._nav_client.server_is_ready():
            return False, 'Nav2 action server not available'

        pose.header.stamp = self.get_clock().now().to_msg()
        self._approach_pose_pub.publish(pose)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        def _feedback_cb(fb_msg):
            feedback = NavigateToObject.Feedback()
            feedback.distance_remaining = fb_msg.feedback.distance_remaining
            goal_handle.publish_feedback(feedback)

        nav_goal_handle = await self._nav_client.send_goal_async(
            nav_goal, feedback_callback=_feedback_cb)
        if not nav_goal_handle.accepted:
            return False, 'Nav2 rejected the goal'
        nav_result = await nav_goal_handle.get_result_async()
        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'succeeded'
        return False, f'Nav2 failed with status {nav_result.status}'

    def _refinement_worthwhile(self, pose):
        """Skip the phase-2 move when the refined pose is where the robot
        already stands (within tolerance)."""
        robot = self._costmap_reader.get_robot_pose(self._global_frame)
        if robot is None:
            return True
        dist = math.hypot(pose.pose.position.x - robot[0],
                          pose.pose.position.y - robot[1])
        return (dist > self._refine_pos_tol)

    async def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = NavigateToObject.Result()

        method = goal.align_method if goal.align_method else self._default_method
        standoff = goal.standoff if goal.standoff > 0.0 else None
        self.get_logger().info(
            f'Goal: object=({goal.obj_x:.3f}, {goal.obj_y:.3f}), '
            f'method={method}')

        # Edge already visible (object close, camera on it)? Go straight in.
        if method == 'nearest_edge':
            pose = self._try_nearest_edge(goal.obj_x, goal.obj_y, standoff)
            if pose is not None:
                self.get_logger().info('Edge visible, single-phase approach')
                ok, msg = await self._navigate(goal_handle, pose)
                if ok:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                result.success = ok
                result.message = ('Reached edge-aligned approach pose'
                                  if ok else msg)
                return result
            if not self._two_phase:
                self.get_logger().warn(
                    'Edge not visible and two_phase disabled, '
                    'falling back to face_target only')

        # Phase 1: walk up facing the object so the camera marks the table
        pose1 = self._face_target_pose(goal.obj_x, goal.obj_y, standoff)
        if pose1 is None:
            self.get_logger().error('Approach pose computation failed')
            goal_handle.abort()
            result.success = False
            result.message = 'Failed to compute approach pose'
            return result
        self.get_logger().info('Phase 1: face_target toward object')
        ok, msg = await self._navigate(goal_handle, pose1)
        if not ok:
            goal_handle.abort()
            result.success = False
            result.message = f'Phase 1 (face_target) failed: {msg}'
            return result

        if method != 'nearest_edge' or not self._two_phase:
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached approach pose (method: face_target)'
            return result

        # Phase 2: camera now sees the table edge -> PCA refinement
        pose2 = self._try_nearest_edge(goal.obj_x, goal.obj_y, standoff)
        if pose2 is None:
            self.get_logger().warn(
                'Phase 2: edge still not detected, staying at face_target pose')
            goal_handle.succeed()
            result.success = True
            result.message = ('Reached face_target pose; table edge not '
                              'detected for refinement')
            return result
        if not self._refinement_worthwhile(pose2):
            self.get_logger().info('Phase 2: already at refined pose')
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached edge-aligned approach pose'
            return result

        self.get_logger().info('Phase 2: refining with nearest_edge')
        ok, msg = await self._navigate(goal_handle, pose2)
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result.success = ok
        result.message = ('Reached edge-aligned approach pose (two-phase)'
                          if ok else f'Phase 2 (nearest_edge) failed: {msg}')
        return result

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    def _make_marker(self, mid, mtype, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'nav_commander'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _color(r, g, b, a=1.0):
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    @staticmethod
    def _pt(x, y, z=0.05):
        return Point(x=float(x), y=float(y), z=float(z))

    def _publish_debug_markers(self, frame_id):
        dbg = self._computer.last_debug
        if not self._debug or dbg is None:
            return

        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)

        cell = dbg.get('cell_size', 0.05)

        # Occupied cells in the ROI (red points)
        occ = dbg.get('occupied_points')
        if occ is not None and len(occ) > 0:
            m = self._make_marker(1, Marker.POINTS, frame_id)
            m.scale.x = m.scale.y = cell * 0.8
            m.color = self._color(1.0, 0.2, 0.2, 0.5)
            m.points = [self._pt(x, y, 0.02) for x, y in occ]
            arr.markers.append(m)

        # Front-edge strip used for PCA (yellow points)
        front = dbg.get('front_points')
        if front is not None and len(front) > 0:
            m = self._make_marker(2, Marker.POINTS, frame_id)
            m.scale.x = m.scale.y = cell * 1.2
            m.color = self._color(1.0, 1.0, 0.0, 0.9)
            m.points = [self._pt(x, y, 0.04) for x, y in front]
            arr.markers.append(m)

        # PCA edge direction through the strip mean (cyan line)
        if 'mu' in dbg and 'e1' in dbg:
            mu, e1 = dbg['mu'], dbg['e1']
            half = max(dbg.get('half_extent', 0.4), 0.2)
            m = self._make_marker(3, Marker.LINE_LIST, frame_id)
            m.scale.x = 0.02
            m.color = self._color(0.0, 1.0, 1.0)
            m.points = [
                self._pt(mu[0] - e1[0] * half, mu[1] - e1[1] * half),
                self._pt(mu[0] + e1[0] * half, mu[1] + e1[1] * half),
            ]
            arr.markers.append(m)

        # Edge normal toward the robot (green arrow)
        if 'normal' in dbg and 'obj_on_edge' in dbg:
            n, oe = dbg['normal'], dbg['obj_on_edge']
            m = self._make_marker(4, Marker.ARROW, frame_id)
            m.scale.x, m.scale.y, m.scale.z = 0.03, 0.06, 0.06
            m.color = self._color(0.0, 1.0, 0.2)
            m.points = [
                self._pt(oe[0], oe[1]),
                self._pt(oe[0] + n[0] * 0.4, oe[1] + n[1] * 0.4),
            ]
            arr.markers.append(m)

        # Object (magenta sphere) and its projection on the edge (blue sphere)
        ox, oy = dbg['object']
        m = self._make_marker(5, Marker.SPHERE, frame_id)
        m.pose.position = self._pt(ox, oy, 0.1)
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color = self._color(1.0, 0.0, 1.0)
        arr.markers.append(m)
        if 'obj_on_edge' in dbg:
            oe = dbg['obj_on_edge']
            m = self._make_marker(6, Marker.SPHERE, frame_id)
            m.pose.position = self._pt(oe[0], oe[1], 0.1)
            m.scale.x = m.scale.y = m.scale.z = 0.10
            m.color = self._color(0.2, 0.4, 1.0)
            arr.markers.append(m)

        # Approach pose arrow showing the final yaw (orange)
        if 'approach' in dbg and 'yaw' in dbg:
            ap, yaw = dbg['approach'], dbg['yaw']
            m = self._make_marker(7, Marker.ARROW, frame_id)
            m.scale.x, m.scale.y, m.scale.z = 0.04, 0.08, 0.08
            m.color = self._color(1.0, 0.6, 0.0)
            m.points = [
                self._pt(ap[0], ap[1]),
                self._pt(ap[0] + math.cos(yaw) * 0.35,
                         ap[1] + math.sin(yaw) * 0.35),
            ]
            arr.markers.append(m)

        # Text panel with the numbers, floating above the object
        lines = [f"method: {dbg.get('method', 'none')}"]
        if occ is not None:
            n_front = 0 if front is None else len(front)
            lines.append(f'cells: {len(occ)} occ / {n_front} front')
        if 'eig_ratio' in dbg:
            lines.append(f"eig ratio: {dbg['eig_ratio']:.1f}")
        if 'yaw' in dbg:
            lines.append(f"yaw: {math.degrees(dbg['yaw']):.1f} deg")
        lines.append(f"standoff: {dbg['standoff']:.2f} m")
        if 'fail_reason' in dbg:
            lines.append(f"edge fail: {dbg['fail_reason']}")

        # Dark backing box first, so the white text renders on top of it
        text_z = 0.7
        bg = self._make_marker(9, Marker.CUBE, frame_id)
        bg.pose.position = self._pt(ox, oy, text_z)
        bg.scale.x = max(len(line) for line in lines) * 0.075
        bg.scale.y = len(lines) * 0.16
        bg.scale.z = 0.01
        bg.color = self._color(0.0, 0.0, 0.0, 0.65)
        arr.markers.append(bg)

        m = self._make_marker(8, Marker.TEXT_VIEW_FACING, frame_id)
        m.pose.position = self._pt(ox, oy, text_z + 0.02)
        m.scale.z = 0.12
        m.color = self._color(1.0, 1.0, 1.0)
        m.text = '\n'.join(lines)
        arr.markers.append(m)

        self._debug_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = NavCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
