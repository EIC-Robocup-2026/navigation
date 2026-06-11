#!/usr/bin/env python3
"""NavCommander: action server that turns raw object positions into Nav2 goals.

The table edge is only marked into the LOCAL costmap (STVL from the ZED head
camera), and only once the camera actually sees the table. So navigation is
two-phase:

  1. approach — walk to a standoff point near the object. The pose is
     grounded by the planner first: ComputePathToPose plans to the geometric
     standoff point, the path's last pose becomes the goal position (NavFn's
     tolerance may snap an unreachable point to the nearest plannable one),
     and the yaw is set to face the object from there. While driving, once
     the robot is within refine_trigger_distance of the goal and roughly
     facing the object, the PCA edge fit is attempted on the local costmap.
  2. nearest_edge — as soon as the fit succeeds (usually mid-approach; phase
     1 is canceled), navigate to the refined pose: directly in front of the
     object, heading perpendicular into the table edge.

Phase 1 is skipped when the edge is already visible at goal time, and the
PCA is retried on arrival if it never triggered mid-drive. The local costmap
lives in the odom frame, so the PCA runs in odom and the result is
transformed back to map before being sent to Nav2.

RViz integration: the "Publish Point" tool (/clicked_point) places an object
and triggers the full pipeline; debug markers visualize every algorithm stage
and can be toggled at runtime with `ros2 param set /nav_commander debug true`.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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
        self.declare_parameter('edge_fit_radius', 0.4)
        self.declare_parameter('edge_inlier_dist', 0.10)
        self.declare_parameter('default_method', 'nearest_edge')
        self.declare_parameter('fallback_method', 'face_target')
        self.declare_parameter('two_phase', True)
        self.declare_parameter('refine_position_tolerance', 0.08)
        self.declare_parameter('refine_yaw_tolerance_deg', 8.0)
        # Mid-approach refinement: once within this distance of the phase-1
        # goal AND roughly facing the object, run the PCA and switch to the
        # edge-aligned pose without waiting for phase 1 to finish.
        self.declare_parameter('refine_trigger_distance', 0.4)
        self.declare_parameter('refine_trigger_yaw_deg', 30.0)
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('compute_path_action', 'compute_path_to_pose')
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
            'edge_fit_radius': self.get_parameter('edge_fit_radius').value,
            'edge_inlier_dist': self.get_parameter('edge_inlier_dist').value,
        }
        self._default_method = self.get_parameter('default_method').value
        self._two_phase = self.get_parameter('two_phase').value
        self._refine_pos_tol = self.get_parameter('refine_position_tolerance').value
        self._refine_yaw_tol = math.radians(
            self.get_parameter('refine_yaw_tolerance_deg').value)
        self._refine_trigger_dist = self.get_parameter('refine_trigger_distance').value
        self._refine_trigger_yaw = math.radians(
            self.get_parameter('refine_trigger_yaw_deg').value)
        self._global_frame = self.get_parameter('global_frame').value
        self._planner_id = self.get_parameter('planner_id').value

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
        # Used to ground the phase-1 pose: plan to the geometric standoff
        # point, take the path's last pose, face the object from there.
        self._path_client = ActionClient(
            self, ComputePathToPose,
            self.get_parameter('compute_path_action').value,
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

    async def _ground_phase1_pose(self, pose, ox, oy):
        """Ground the geometric standoff pose with the planner: plan to it,
        move the goal to the path's last pose (NavFn tolerance may have
        snapped an unreachable point to the nearest plannable one), and face
        the object from that actual position. Returns the pose unchanged
        when planning is unavailable or fails — its yaw already faces the
        object from the geometric point."""
        if not self._path_client.server_is_ready():
            self.get_logger().warn(
                'compute_path_to_pose not available, using geometric pose')
            return pose
        path_goal = ComputePathToPose.Goal()
        path_goal.goal = pose
        path_goal.planner_id = self._planner_id
        path_goal.use_start = False
        handle = await self._path_client.send_goal_async(path_goal)
        if not handle.accepted:
            return pose
        result = await handle.get_result_async()
        if (result.status != GoalStatus.STATUS_SUCCEEDED
                or len(result.result.path.poses) == 0):
            self.get_logger().warn(
                'Phase-1 path planning failed, using geometric pose')
            return pose
        end = result.result.path.poses[-1].pose.position
        if math.hypot(ox - end.x, oy - end.y) < 1e-3:
            return pose
        yaw = math.atan2(oy - end.y, ox - end.x)
        self.get_logger().info(
            f'Phase-1 pose grounded by planner: path end '
            f'({end.x:.3f}, {end.y:.3f}), heading to object '
            f'{math.degrees(yaw):.1f} deg')
        return ApproachPoseComputer._build_pose_stamped(end.x, end.y, yaw)

    async def _navigate(self, goal_handle, pose, refine_check=None):
        """Send one pose to Nav2 and await the outcome.

        refine_check(feedback) may return a better pose mid-drive; when it
        does, this navigation is canceled and the pose is handed back to the
        caller.

        Returns (ok, msg, refined_pose).
        """
        yaw = _yaw_from_quaternion(pose.pose.orientation)
        self.get_logger().info(
            f'Navigating to x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, yaw={math.degrees(yaw):.1f} deg')

        if not self._nav_client.server_is_ready():
            return False, 'Nav2 action server not available', None

        pose.header.stamp = self.get_clock().now().to_msg()
        self._approach_pose_pub.publish(pose)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        state = {'handle': None, 'refined': None, 'last_try': 0.0}

        def _feedback_cb(fb_msg):
            fb = fb_msg.feedback
            feedback = NavigateToObject.Feedback()
            feedback.distance_remaining = fb.distance_remaining
            goal_handle.publish_feedback(feedback)

            if refine_check is None or state['refined'] is not None:
                return
            now = time.monotonic()
            if now - state['last_try'] < 0.5:  # throttle PCA attempts
                return
            state['last_try'] = now
            refined = refine_check(fb)
            if refined is not None:
                state['refined'] = refined
                self.get_logger().info(
                    'Edge alignment found mid-approach, canceling phase 1')
                if state['handle'] is not None:
                    state['handle'].cancel_goal_async()

        nav_goal_handle = await self._nav_client.send_goal_async(
            nav_goal, feedback_callback=_feedback_cb)
        if not nav_goal_handle.accepted:
            return False, 'Nav2 rejected the goal', None
        state['handle'] = nav_goal_handle
        if state['refined'] is not None:  # refine fired before handle was set
            nav_goal_handle.cancel_goal_async()

        nav_result = await nav_goal_handle.get_result_async()
        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'succeeded', state['refined']
        return (False, f'Nav2 failed with status {nav_result.status}',
                state['refined'])

    def _facing_object(self, ox, oy):
        """True when the robot's heading is within refine_trigger_yaw of the
        bearing to the object — i.e. the camera is actually looking at the
        table, so the local costmap marks there can be trusted for the PCA."""
        robot = self._costmap_reader.get_robot_pose_full(self._global_frame)
        if robot is None:
            return False
        bearing = math.atan2(oy - robot[1], ox - robot[0])
        err = math.atan2(math.sin(bearing - robot[2]),
                         math.cos(bearing - robot[2]))
        return abs(err) <= self._refine_trigger_yaw

    def _refinement_worthwhile(self, pose):
        """Skip the phase-2 move only when the robot already stands at the
        refined pose — position AND heading. Phase 1 may have been canceled
        mid-drive, leaving the robot near the spot but pointing wherever it
        was driving; the aligned heading is the whole point of phase 2."""
        robot = self._costmap_reader.get_robot_pose_full(self._global_frame)
        if robot is None:
            return True
        dist = math.hypot(pose.pose.position.x - robot[0],
                          pose.pose.position.y - robot[1])
        target_yaw = _yaw_from_quaternion(pose.pose.orientation)
        dyaw = math.atan2(math.sin(target_yaw - robot[2]),
                          math.cos(target_yaw - robot[2]))
        return dist > self._refine_pos_tol or abs(dyaw) > self._refine_yaw_tol

    async def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = NavigateToObject.Result()

        method = goal.align_method if goal.align_method else self._default_method
        standoff = goal.standoff if goal.standoff > 0.0 else None
        self.get_logger().info(
            f'Goal: object=({goal.obj_x:.3f}, {goal.obj_y:.3f}), '
            f'method={method}')

        # Edge already visible? Go straight in — but only when the robot is
        # actually FACING the object (same gate as the mid-approach trigger):
        # the local costmap can hold stale marks the camera never confirmed,
        # and a PCA on those must not skip the approach phase.
        if method == 'nearest_edge':
            pose = None
            if self._facing_object(goal.obj_x, goal.obj_y):
                pose = self._try_nearest_edge(goal.obj_x, goal.obj_y, standoff)
            else:
                self.get_logger().info(
                    'Not facing the object, skipping direct edge check')
            if pose is not None:
                self.get_logger().info('Edge visible, single-phase approach')
                ok, msg, _ = await self._navigate(goal_handle, pose)
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

        # Phase 1: walk up to a standoff point near the object, facing it.
        # The pose is grounded by the planner (path end + heading to object),
        # and the refine check below switches to the PCA pose as soon as the
        # robot is close and facing the object — usually before phase 1 even
        # finishes.
        pose1 = self._face_target_pose(goal.obj_x, goal.obj_y, standoff)
        if pose1 is not None:
            pose1 = await self._ground_phase1_pose(
                pose1, goal.obj_x, goal.obj_y)
        if pose1 is None:
            self.get_logger().error('Approach pose computation failed')
            goal_handle.abort()
            result.success = False
            result.message = 'Failed to compute approach pose'
            return result

        refine_check = None
        if method == 'nearest_edge' and self._two_phase:
            def refine_check(fb):
                # Trigger only near the goal and roughly facing the object,
                # i.e. when the camera can actually see the table edge.
                if not (1e-3 < fb.distance_remaining
                        <= self._refine_trigger_dist):
                    return None
                cur = fb.current_pose.pose
                yaw = _yaw_from_quaternion(cur.orientation)
                bearing = math.atan2(goal.obj_y - cur.position.y,
                                     goal.obj_x - cur.position.x)
                err = math.atan2(math.sin(bearing - yaw),
                                 math.cos(bearing - yaw))
                if abs(err) > self._refine_trigger_yaw:
                    return None
                return self._try_nearest_edge(
                    goal.obj_x, goal.obj_y, standoff)

        self.get_logger().info('Phase 1: approaching object')
        ok, msg, refined = await self._navigate(
            goal_handle, pose1, refine_check=refine_check)

        if refined is None and not ok:
            goal_handle.abort()
            result.success = False
            result.message = f'Phase 1 (approach) failed: {msg}'
            return result

        if method != 'nearest_edge' or not self._two_phase:
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached approach pose (method: face_target)'
            return result

        # Phase 2: edge-aligned pose, either found mid-approach (phase 1 was
        # canceled for it) or computed now that the robot has arrived.
        pose2 = refined
        if pose2 is None:
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

        self.get_logger().info('Phase 2: navigating to edge-aligned pose')
        ok, msg, _ = await self._navigate(goal_handle, pose2)
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

        # Raycast seed: first occupied cell on the robot->object ray (white)
        if 'ray_seed' in dbg:
            s = dbg['ray_seed']
            m = self._make_marker(10, Marker.SPHERE, frame_id)
            m.pose.position = self._pt(s[0], s[1], 0.06)
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color = self._color(1.0, 1.0, 1.0)
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
        if 'anchor' in dbg:
            lines.append(f"anchor: {dbg['anchor']}")
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
