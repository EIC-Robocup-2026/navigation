#!/usr/bin/env python3
"""Approach-pose computation for object navigation goals.

Pure Python: no ROS2 imports at module level so the math is unit-testable
without a ROS environment. The only ROS types appear inside
_build_pose_stamped, which is the last step of every code path.

Given a raw object position (x, y) in the map frame and the global costmap,
compute a standoff pose directly in front of the object with the robot facing
into the supporting surface (table). The depth camera only marks the table's
front edge reliably, so the edge is recovered from the occupied cells nearest
the robot rather than from the full table outline.
"""

import math

import numpy as np

# Minimum PC1/PC2 variance ratio for the front-edge cluster to count as a line.
EDGE_LINEARITY_RATIO = 5.0
OCCUPIED = 100


class ApproachPoseComputer:
    def __init__(self, config: dict, logger=None):
        self.standoff = float(config['standoff'])
        self.inflation_radius = float(config['inflation_radius'])
        self.search_radius = float(config['search_radius'])
        self.min_occupied_cells = int(config['min_occupied_cells'])
        self.front_edge_tolerance = float(config['front_edge_tolerance'])
        self._logger = logger
        # Method that actually produced the last pose ('nearest_edge' or
        # 'face_target'), for callers that report it in action results.
        self.last_method = None
        # Intermediate results of the last compute() call (numpy arrays and
        # floats only), for debug visualization. Keys appear progressively as
        # the algorithm advances; 'fail_reason' is set when a stage bails out.
        self.last_debug = None

    def _log_info(self, msg):
        if self._logger is not None:
            self._logger.info(msg)

    def _log_warn(self, msg):
        if self._logger is not None:
            self._logger.warn(msg)

    def compute(self, obj_x, obj_y, robot_x, robot_y, method, costmap,
                standoff=None, allow_fallback=True):
        """Compute the approach PoseStamped, or None on failure.

        standoff overrides the configured default when not None (action goals
        may request a per-goal standoff). With allow_fallback=False a failed
        nearest_edge returns None instead of degrading to face_target, so
        callers can stage the two methods themselves (walk up facing the
        object first, refine with PCA once the camera sees the table).

        All coordinates (object, robot, costmap origin) must share one frame;
        the returned pose is in that same frame.
        """
        standoff = self.standoff if standoff is None else float(standoff)
        self.last_debug = {
            'object': (float(obj_x), float(obj_y)),
            'robot': (float(robot_x), float(robot_y)),
            'requested_method': method,
            'standoff': standoff,
        }

        pose = None
        used = method
        if method == 'nearest_edge':
            if costmap is None:
                self._log_warn('nearest_edge requested but no costmap available')
            else:
                pose = self._nearest_edge(
                    obj_x, obj_y, robot_x, robot_y, costmap, standoff)
        elif method != 'face_target':
            self._log_warn(f"unknown align method '{method}', using face_target")

        if pose is None:
            if not allow_fallback and method != 'face_target':
                self.last_method = None
                self.last_debug['method'] = None
                self._log_info(f'{method} failed, no fallback requested')
                return None
            used = 'face_target'
            pose = self._face_target(obj_x, obj_y, robot_x, robot_y, standoff)

        self.last_method = used if pose is not None else None
        self.last_debug['method'] = self.last_method
        self._log_info(f'approach pose computed with method: {used}')
        return pose

    def _nearest_edge(self, ox, oy, rx, ry, costmap, standoff):
        """PCA on the front-edge strip of occupied costmap cells around the
        object. Returns None whenever the data does not support a confident
        edge estimate, so the caller falls back to face_target."""
        info = costmap.info
        res = info.resolution
        if res <= 0.0:
            return None
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        data = np.asarray(costmap.data, dtype=np.int8).reshape(height, width)

        # 1. Crop ROI around the object
        gcx = int((ox - origin_x) / res)
        gcy = int((oy - origin_y) / res)
        r = int(self.search_radius / res)
        col_min, col_max = max(0, gcx - r), min(width, gcx + r)
        row_min, row_max = max(0, gcy - r), min(height, gcy + r)
        if col_min >= col_max or row_min >= row_max:
            self.last_debug['fail_reason'] = 'object ROI outside costmap'
            return None
        roi = data[row_min:row_max, col_min:col_max]

        # 2. Occupied cells in world coords
        occ_rows, occ_cols = np.where(roi == OCCUPIED)
        if len(occ_rows) < self.min_occupied_cells:
            self.last_debug['fail_reason'] = (
                f'{len(occ_rows)} occupied cells in ROI '
                f'(min {self.min_occupied_cells})')
            return None
        world_x = (occ_cols + col_min) * res + origin_x
        world_y = (occ_rows + row_min) * res + origin_y
        points = np.column_stack([world_x, world_y])
        self.last_debug['occupied_points'] = points
        self.last_debug['cell_size'] = res

        # 3. Front edge: cells nearest the robot. Back/sides of the table are
        # often missing from the depth projection, so only this strip is used.
        dists = np.linalg.norm(points - [rx, ry], axis=1)
        d_min = dists.min()
        front_mask = dists <= (d_min + self.front_edge_tolerance)
        front_points = points[front_mask]
        self.last_debug['front_points'] = front_points
        if len(front_points) < 2:
            self.last_debug['fail_reason'] = (
                f'{len(front_points)} front-edge cells (min 2)')
            return None

        # 4. PCA: PC1 is the edge direction
        mu = front_points.mean(axis=0)
        centered = front_points - mu
        C = (centered.T @ centered) / len(front_points)
        eigenvalues, eigenvectors = np.linalg.eigh(C)
        e1 = eigenvectors[:, 1]
        eig_ratio = eigenvalues[1] / max(eigenvalues[0], 1e-9)
        self.last_debug['mu'] = mu
        self.last_debug['e1'] = e1
        self.last_debug['eig_ratio'] = float(eig_ratio)
        if eig_ratio < EDGE_LINEARITY_RATIO:
            self.last_debug['fail_reason'] = (
                f'edge not linear: eig ratio {eig_ratio:.1f} '
                f'< {EDGE_LINEARITY_RATIO}')
            return None  # cluster is not clearly linear

        # 5. Edge normal pointing toward the robot
        n1 = np.array([-e1[1], e1[0]])
        robot_dir = np.array([rx, ry]) - mu
        normal = n1 if np.dot(n1, robot_dir) > 0 else -n1

        # 6. Project the object onto the edge line so the approach point is in
        # front of the OBJECT, not the center of the detected edge strip.
        obj_vec = np.array([ox, oy]) - mu
        proj_dist = np.dot(obj_vec, e1)
        half_extent = np.max(np.abs(np.dot(front_points - mu, e1)))
        proj_clamped = np.clip(proj_dist, -half_extent, half_extent)
        obj_on_edge = mu + proj_clamped * e1

        # 7. Standoff pose, yaw facing INTO the table
        approach = obj_on_edge + (self.inflation_radius + standoff) * normal
        yaw = math.atan2(-normal[1], -normal[0])
        self.last_debug.update({
            'normal': normal,
            'obj_on_edge': obj_on_edge,
            'half_extent': float(half_extent),
            'approach': approach,
            'yaw': float(yaw),
        })
        return self._build_pose_stamped(approach[0], approach[1], yaw)

    def _face_target(self, ox, oy, rx, ry, standoff):
        """Costmap-free fallback: stand off along the robot→object line."""
        vec = np.array([ox - rx, oy - ry])
        norm = np.linalg.norm(vec)
        if norm < 1e-6:
            return None  # robot already on the object, direction undefined
        unit_vec = vec / norm
        approach = np.array([ox, oy]) - unit_vec * standoff
        yaw = math.atan2(oy - approach[1], ox - approach[0])
        self.last_debug.update({'approach': approach, 'yaw': float(yaw)})
        return self._build_pose_stamped(approach[0], approach[1], yaw)

    @staticmethod
    def _build_pose_stamped(x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        from tf_transformations import quaternion_from_euler

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
