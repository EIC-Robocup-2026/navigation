# walkie-navigation — NavigateToObject

Object approach-pose system for Walkie. The walkie-agent sends navigation
goals as raw object positions `(x, y)` with no heading; without help the robot
arrives facing a random direction. The `nav_commander` node wraps Nav2 behind
a `/navigate_to_object` action that places the robot at a standoff pose
**directly in front of the object, facing perpendicular into the table edge**.

```
walkie-agent ──(obj x, y)──▶ /navigate_to_object          ┌─ /local_costmap/costmap (STVL, odom frame)
                                  │                       ├─ /amcl_pose + TF
                                  ▼                       │
                            nav_commander  ◀──────────────┘
                                  │  PCA edge fit (approach_pose.py)
                                  ▼
                         /navigate_to_pose (Nav2)
```

## Why two phases

The table edge is only marked into the **local** costmap by the ZED head
camera (STVL layer), and only once the camera actually looks at it. So the
edge data needed to compute the final heading usually does not exist when the
goal arrives:

1. **Phase 1 — approach facing the object.** A standoff point on the
   robot→object line is computed (`face_target`), then *grounded by the
   planner*: `ComputePathToPose` plans to it, the path's last pose becomes
   the goal position (NavFn's tolerance may snap an unreachable point to the
   nearest plannable one), and the yaw is set to face the object from there.
   Driving this leg points the camera at the table, so STVL marks the edge.
2. **Phase 2 — edge alignment.** Once the object is within
   `refine_trigger_distance` (straight-line) AND the robot faces it within
   `refine_trigger_yaw_deg`, the edge fit runs on the live costmap. On
   success phase 1 is **canceled mid-drive** and the robot diverts to the
   refined pose. If the trigger never fires, the fit is retried on arrival;
   if the edge is still not found, the goal succeeds at the face_target pose.

Phase 1 is skipped entirely when the robot already faces the object and the
edge fits immediately (single-phase). The facing gate also protects against
trusting **stale** costmap marks the camera never confirmed.

## Edge fit algorithm (`robot_navigation/approach_pose.py`)

Pure Python + numpy, no ROS imports at module level (unit-testable without a
ROS environment). The costmap marks the whole visible tabletop as a filled
blob, so the edge is recovered from its outline:

1. **ROI crop** around the object (`search_radius`).
2. **Rim extraction** — occupied cells (`== 100`) with at least one free
   4-neighbour. Interior cells only smear the fit.
3. **Raycast anchor** — Bresenham walk from robot to object; the first
   occupied cell is the rim the robot must stand in front of. Anchoring on
   the globally nearest cell could latch onto a chair/wall inside the ROI.
   Fallback when the ray misses: rim cell nearest the robot.
4. **Iterative line fit** — PCA seeded with rim cells within
   `edge_fit_radius` of the anchor; each pass keeps only cells within
   `edge_inlier_dist` *perpendicular* of the line, so corner side-edges and
   clutter drop out. Then the inlier set is extended along the full visible
   rim and refit for a stable direction.
5. **Quality gate** — PC1/PC2 eigenvalue ratio ≥ 10, else the fit is
   rejected (caller falls back to `face_target`).
6. **Pose** — object projected onto the edge line (clamped to the visible
   extent, so the approach is in front of the *object*, not the edge
   center), standoff = `inflation_radius + standoff` along the robot-facing
   normal, yaw faces into the table.

### Multi-sample aggregation

Every edge-pose estimate aggregates up to `refine_samples` fits (each gated
on a fresh costmap update, `refine_sample_interval` as timeout fallback).
Outliers are rejected in two stages — position consensus around the median,
then heading consensus referenced on the position inliers — and the
survivors are averaged (circular mean for yaw). No consensus rejects the
whole batch, so one noisy STVL update can never steer the approach.

## Action interface

```bash
ros2 action send_goal /navigate_to_object robot_navigation/action/NavigateToObject \
  "{obj_x: 1.0, obj_y: -3.0, align_method: '', standoff: 0.0}" --feedback
```

| Goal field | Meaning |
|---|---|
| `obj_x`, `obj_y` | Object position in the map frame |
| `align_method` | `''` = default (`nearest_edge`), or `face_target` to skip edge alignment |
| `standoff` | Per-goal standoff override in m; `0.0` = use the configured default |

Result: `success`, `message` (which method ended up being used). Feedback:
`distance_remaining` forwarded from Nav2.

## Configuration (`config/walkie_nav.yaml`)

| Parameter | Role |
|---|---|
| `standoff`, `inflation_radius` | Final distance from the edge = `inflation_radius + standoff` |
| `search_radius`, `min_occupied_cells` | ROI size / minimum occupied cells to attempt the fit |
| `edge_fit_radius`, `edge_inlier_dist` | Line-fit seed neighbourhood / perpendicular inlier gate |
| `two_phase` | Disable to always use plain `face_target` after phase 1 |
| `refine_trigger_distance`, `refine_trigger_yaw_deg` | Mid-approach trigger: object within straight-line distance AND robot facing it. Keep the yaw gate inside the camera half-FOV (ZED ≈ ±40°) |
| `refine_position_tolerance`, `refine_yaw_tolerance_deg` | Skip the phase-2 move only when the robot already matches the refined pose in position AND heading |
| `refine_samples`, `refine_sample_interval` | Multi-sample aggregation count / per-sample wait |
| `planner_id` | Planner used to ground the phase-1 pose |
| `costmap_topic`, `robot_pose_topic`, `clicked_point_topic` | Topic remaps |
| `debug` | Publish debug markers; runtime-toggleable (`ros2 param set /nav_commander debug true`) |

## RViz tooling

- **Place object and go**: toolbar **Publish Point** → click the map next to
  a table. The click (`/clicked_point`) is treated as an object position and
  runs the full pipeline.
- **Approach pose**: add a *Pose* display on `/nav_commander/approach_pose`.
- **Debug markers** (`debug: true` or launch arg `nav_debug:=true`): add a
  *MarkerArray* display on `/nav_commander/debug_markers`:

  | Marker | Meaning |
  |---|---|
  | Red points | Occupied cells in the ROI |
  | Yellow points | Rim inliers used by the line fit |
  | Cyan line | Fitted edge direction (PC1) |
  | Green arrow | Edge normal toward the robot |
  | White sphere | Raycast anchor (first occupied cell on the robot→object ray) |
  | Magenta / blue spheres | Object / object projected onto the edge |
  | Orange arrow | Final approach position + yaw |
  | Text panel | method, anchor mode, cell counts, eig ratio, yaw, standoff, failure reason |

  With `debug: true`, every aggregation sample also logs
  `edge sample i/N: x.. y.. yaw.. (eig ..)`.

## Files

| File | Role |
|---|---|
| `robot_navigation/nav_commander.py` | Action server, two-phase orchestration, multi-sample aggregation, debug markers |
| `robot_navigation/approach_pose.py` | Pure-Python edge fit + pose math |
| `robot_navigation/costmap_reader.py` | Costmap/pose cache, frame-aware TF helpers (local costmap is in odom; goals are in map) |
| `action/NavigateToObject.action` | Action definition |
| `config/walkie_nav.yaml` | All parameters |

`nav_commander` is launched by `localization_Nav2_real.launch.py`,
`localization_Nav2_sim.launch.py` and `nav2_loopback.launch.py`.

## Testing without the robot

```bash
ros2 launch robot_navigation nav2_loopback.launch.py nav_debug:=true
```

Set an initial pose (2D Pose Estimate), then click **Publish Point** next to
a mapped wall/furniture blob. Note: loopback has no camera, so the local
costmap usually has no STVL marks — phase 2 then reports "edge not detected"
and finishes at the face_target pose, which is the designed fallback. The
full two-phase refinement needs Gazebo or the real robot.

Notes for the real robot: `approach_pose.py` deliberately avoids
`tf_transformations` (its `transforms3d` dependency breaks under NumPy 2,
which shadows the apt NumPy on the Jetson via user-site pip packages).
