#!/usr/bin/env python3
"""Approach-pose computation for object navigation goals.

Pure Python: no ROS2 imports at module level so the math is unit-testable
without a ROS environment. The only ROS types appear inside
_build_pose_stamped, which is the last step of every code path.

Given a raw object position (x, y) in the map frame and the global costmap,
compute a standoff pose directly in front of the object with the robot facing
into the supporting surface (table). The depth camera only marks the table's
front edge reliably, so the edge is recovered from a distance band of occupied
cells anchored on the first obstacle along the robot->object ray, rather than
from the full table outline.
"""

import math

import numpy as np

# Minimum PC1/PC2 variance ratio for the fitted rim strip to count as a line.
# The iterative fit produces a 1-2 cell thick strip, which scores far higher
# than the old distance-band cluster, so the gate is stricter.
EDGE_LINEARITY_RATIO = 10.0
OCCUPIED = 100
LINE_FIT_ITERATIONS = 3


class ApproachPoseComputer:
    def __init__(self, config: dict, logger=None):
        self.standoff = float(config['standoff'])
        self.inflation_radius = float(config['inflation_radius'])
        self.search_radius = float(config['search_radius'])
        self.min_occupied_cells = int(config['min_occupied_cells'])
        # Rim line fit: cells within edge_fit_radius of the raycast anchor
        # seed the fit; cells within edge_inlier_dist perpendicular of the
        # fitted line count as inliers.
        self.edge_fit_radius = float(config.get('edge_fit_radius', 0.4))
        self.edge_inlier_dist = float(config.get('edge_inlier_dist', 0.10))
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
        """Line fit on the table's front rim.

        The costmap marks the whole visible tabletop as a filled blob, not a
        thin outline, so the edge is recovered as: boundary cells (occupied
        with a free 4-neighbour) -> raycast anchor on the rim between robot
        and object -> iterative local line fit around the anchor
        (perpendicular outliers such as side edges and clutter drop out) ->
        extend along the full visible rim for a stable direction. Returns
        None whenever the data does not support a confident edge estimate,
        so the caller falls back to face_target."""
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

        # 3. Rim (boundary) cells: occupied with at least one free
        # 4-neighbour. The tabletop is a filled blob in the costmap; the line
        # we want is its outline, and interior cells only smear the fit.
        occ = data == OCCUPIED
        up = np.zeros_like(occ)
        down = np.zeros_like(occ)
        left = np.zeros_like(occ)
        right = np.zeros_like(occ)
        up[:-1, :] = occ[1:, :]
        down[1:, :] = occ[:-1, :]
        left[:, 1:] = occ[:, :-1]
        right[:, :-1] = occ[:, 1:]
        boundary = occ & ~(up & down & left & right)
        b_rows, b_cols = np.where(boundary[row_min:row_max, col_min:col_max])
        if len(b_rows) < 2:
            self.last_debug['fail_reason'] = (
                f'{len(b_rows)} rim cells in ROI (min 2)')
            return None
        boundary_points = np.column_stack([
            (b_cols + col_min) * res + origin_x,
            (b_rows + row_min) * res + origin_y,
        ])

        # 4. Anchor on the rim BETWEEN robot and object (first occupied cell
        # on the robot->object ray) — the globally nearest cell can belong to
        # a chair/wall that has nothing to do with the table. Fall back to
        # the rim cell nearest the robot when the ray hits nothing.
        seed = self._raycast_first_occupied(
            data,
            int((rx - origin_x) / res), int((ry - origin_y) / res),
            gcx, gcy)
        if seed is not None:
            anchor_xy = np.array(
                [seed[0] * res + origin_x, seed[1] * res + origin_y])
            self.last_debug['ray_seed'] = tuple(anchor_xy)
            self.last_debug['anchor'] = 'raycast'
        else:
            b_dists = np.linalg.norm(boundary_points - [rx, ry], axis=1)
            anchor_xy = boundary_points[np.argmin(b_dists)]
            self.last_debug['anchor'] = 'nearest'

        # 5. Iterative line fit, seeded near the anchor. Each pass refits on
        # the cells within edge_inlier_dist perpendicular of the line, so
        # corner-adjacent side-edge cells and clutter fall out as outliers.
        local = boundary_points[
            np.linalg.norm(boundary_points - anchor_xy, axis=1)
            <= self.edge_fit_radius]
        if len(local) < 2:
            self.last_debug['fail_reason'] = (
                f'{len(local)} rim cells near anchor (min 2)')
            return None
        sel = local
        for _ in range(LINE_FIT_ITERATIONS):
            mu, e1, _ = self._pca(sel)
            n_hat = np.array([-e1[1], e1[0]])
            sel = local[np.abs((local - mu) @ n_hat) <= self.edge_inlier_dist]
            if len(sel) < 2:
                self.last_debug['fail_reason'] = 'line fit collapsed'
                return None
        # Extend along the full visible rim: a longer baseline stabilizes
        # the direction far beyond what the local neighbourhood provides.
        front_points = boundary_points[
            np.abs((boundary_points - mu) @ n_hat) <= self.edge_inlier_dist]
        mu, e1, eigenvalues = self._pca(front_points)
        eig_ratio = eigenvalues[1] / max(eigenvalues[0], 1e-9)
        self.last_debug['front_points'] = front_points
        self.last_debug['mu'] = mu
        self.last_debug['e1'] = e1
        self.last_debug['eig_ratio'] = float(eig_ratio)
        if eig_ratio < EDGE_LINEARITY_RATIO:
            self.last_debug['fail_reason'] = (
                f'edge not linear: eig ratio {eig_ratio:.1f} '
                f'< {EDGE_LINEARITY_RATIO}')
            return None  # rim strip is not clearly a line

        # 6. Edge normal pointing toward the robot
        n1 = np.array([-e1[1], e1[0]])
        robot_dir = np.array([rx, ry]) - mu
        normal = n1 if np.dot(n1, robot_dir) > 0 else -n1

        # 7. Project the object onto the edge line so the approach point is in
        # front of the OBJECT, not the center of the detected edge strip.
        obj_vec = np.array([ox, oy]) - mu
        proj_dist = np.dot(obj_vec, e1)
        half_extent = np.max(np.abs(np.dot(front_points - mu, e1)))
        proj_clamped = np.clip(proj_dist, -half_extent, half_extent)
        obj_on_edge = mu + proj_clamped * e1

        # 8. Standoff pose, yaw facing INTO the table
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
    def _pca(pts):
        """Mean, principal direction (PC1) and eigenvalues of a 2D point
        set. eigenvalues are ascending (eigh), so [1] is the line variance."""
        mu = pts.mean(axis=0)
        centered = pts - mu
        C = (centered.T @ centered) / len(pts)
        eigenvalues, eigenvectors = np.linalg.eigh(C)
        return mu, eigenvectors[:, 1], eigenvalues

    @staticmethod
    def _raycast_first_occupied(data, c0, r0, c1, r1):
        """Bresenham walk over the grid from (c0, r0) toward (c1, r1) in
        (col, row) indices; returns the first occupied cell as (col, row), or
        None if the ray reaches the target without hitting one. Cells outside
        the grid (robot outside a rolling local costmap) are skipped."""
        height, width = data.shape
        dc, dr = abs(c1 - c0), abs(r1 - r0)
        sc = 1 if c1 >= c0 else -1
        sr = 1 if r1 >= r0 else -1
        err = dc - dr
        c, r = c0, r0
        while True:
            if 0 <= c < width and 0 <= r < height and data[r, c] == OCCUPIED:
                return c, r
            if c == c1 and r == r1:
                return None
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

    @staticmethod
    def _build_pose_stamped(x, y, yaw):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        # Planar yaw-only quaternion, inline on purpose: tf_transformations
        # pulls in transforms3d, whose apt version calls np.maximum_sctype
        # and crashes under NumPy 2 (as found on the robot's Jetson).
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose
