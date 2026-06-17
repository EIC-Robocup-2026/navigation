// Compute the in-place rotation (relative yaw) that points the robot's front at
// the direction it is about to TRAVEL — a point sampled a short distance along
// the planned path, falling back to the straight-line bearing to the goal when
// no path is available.
//
// Why this exists
// ---------------
// Walkie is a holonomic omnibot, so MPPI has no kinematic penalty for driving
// backward. Over its short prediction horizon (~2.8 s), when the goal is behind
// the robot it is "cheaper" for MPPI to reverse the whole way than to pay the
// upfront cost of yawing ~180 deg to face forward. The result: the robot crabs
// backward to far goals, with its forward-mounted sensors pointing the wrong way.
//
// This node turns the rotate-to-face step into a discrete BT action that runs
// ONCE per goal, before FollowPath: it reads the freshly planned {path}, finds
// the point ~forward_sampling_distance ahead, and outputs the spin needed to
// face it. A downstream Spin executes it, then MPPI takes over already pointed
// forward and simply drives.
//
// Path heading vs goal bearing
// ----------------------------
// When a path is present we face the PATH heading, so the spin bends correctly
// through a doorway or around furniture instead of facing a wall the path routes
// around. When the path is empty/missing (e.g. the preamble plan transiently
// failed because the costmap or TF wasn't ready yet), we DON'T give up and skip
// the spin — we fall back to the straight-line bearing to the {goal}. This is
// what makes the pre-rotation fire reliably instead of "sometimes".
//
// Rotation-clearance gate (the "boxed-in start" guard)
// ----------------------------------------------------
// If the robot starts inside a tight space (a doorway, between tables), there is
// no room to rotate the footprint in place: a commanded Spin collides, aborts,
// and the robot stalls / drops into recovery before it has even moved. So before
// asking for a spin we check the LOCAL costmap for obstacles within
// rotation_clearance_radius of the base. If the robot is boxed in we skip the
// pre-rotation entirely (spin_dist = 0) and let MPPI drive forward out of the
// gap — MPPI handles tight spaces fine and re-faces nothing it doesn't need to.
// The check fails OPEN: a missing costmap / TF never suppresses the spin.
//
// Gating (so short moves keep their freedom to reverse):
//   - travel distance < min_path_length  -> don't spin (short reposition)
//   - |correction| < min_spin            -> don't spin (already aligned)
//   - |correction| > max_spin            -> don't spin (defaults off at ~pi)
//   - no room to rotate in place          -> don't spin (boxed in, MPPI drives out)
// "travel distance" is the remaining path length when a path is used, else the
// straight-line distance to the goal. When gated off, spin_dist is set to 0.0 so
// a downstream Spin no-ops, and the node still returns SUCCESS so navigation
// proceeds. The node ALWAYS logs its decision and which source it used, so a
// "why didn't it spin?" question is answered straight from the bt_navigator log.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_navigation
{

class ComputePathHeadingSpin : public BT::SyncActionNode
{
public:
  ComputePathHeadingSpin(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf),
    transform_tolerance_(0.1),
    global_frame_("map"),
    robot_base_frame_("base_footprint")
  {
    node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_->get_parameter("transform_tolerance", transform_tolerance_);
    node_->get_parameter("global_frame", global_frame_);
    node_->get_parameter("robot_base_frame", robot_base_frame_);

    // Local costmap = the rolling obstacle window around the robot (live laser /
    // STVL marks, same source the Spin behavior collides against). Latched QoS
    // means the last frame is ready well before the first goal. Used only by the
    // rotation-clearance gate; if it never arrives that gate fails open.
    local_costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      node_, "/local_costmap/costmap_raw");
  }

  ComputePathHeadingSpin() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>(
        "path", "Planned path whose initial heading the robot should face "
        "(falls back to 'goal' bearing when empty/missing)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Goal pose — used for the bearing fallback when no path"),
      BT::InputPort<double>(
        "min_path_length", 3.0,
        "Don't spin if the travel distance is shorter than this (m) — short "
        "moves keep their freedom to reverse"),
      BT::InputPort<double>(
        "forward_sampling_distance", 1.2,
        "Sample the path this far (m) ahead of the robot to read travel heading"),
      BT::InputPort<double>(
        "min_spin", 0.25,
        "Don't spin for corrections smaller than this (rad) — already aligned"),
      BT::InputPort<double>(
        "max_spin", 3.2,
        "Don't spin for corrections larger than this (rad) — default ~pi keeps "
        "it effectively off so even near-reversals get rotated"),
      BT::InputPort<bool>(
        "check_rotation_clearance", true,
        "Skip the pre-rotation when the robot is boxed in (no room to rotate "
        "in place) and let MPPI drive out forward instead"),
      BT::InputPort<double>(
        "rotation_clearance_radius", 0.40,
        "An obstacle within this radius (m) of the base means the robot can't "
        "rotate in place — set ~ the footprint circumscribed radius + margin"),
      BT::InputPort<int>(
        "lethal_cost_threshold", nav2_costmap_2d::LETHAL_OBSTACLE,
        "Local-costmap cost at/above which a cell blocks rotation (254 = lethal)"),
      BT::OutputPort<double>(
        "spin_dist", "Relative yaw (rad) to rotate to face the path "
        "(0 when the spin is gated off)"),
      BT::OutputPort<bool>(
        "do_spin", "False when the spin is gated off, so the BT can skip Spin"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped current;
    if (!nav2_util::getCurrentPose(
        current, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
      gateOff("current robot pose unavailable", true);
      return BT::NodeStatus::SUCCESS;
    }

    double min_path_length = 3.0, forward_sampling_distance = 1.2;
    double min_spin = 0.25, max_spin = 3.2;
    bool check_rotation_clearance = true;
    double rotation_clearance_radius = 0.40;
    int lethal_cost_threshold = nav2_costmap_2d::LETHAL_OBSTACLE;
    getInput("min_path_length", min_path_length);
    getInput("forward_sampling_distance", forward_sampling_distance);
    getInput("min_spin", min_spin);
    getInput("max_spin", max_spin);
    getInput("check_rotation_clearance", check_rotation_clearance);
    getInput("rotation_clearance_radius", rotation_clearance_radius);
    getInput("lethal_cost_threshold", lethal_cost_threshold);

    const double rx = current.pose.position.x;
    const double ry = current.pose.position.y;

    // Resolve a target point to face plus the travel distance, preferring the
    // planned path and falling back to the goal bearing.
    double target_x = 0.0, target_y = 0.0, travel_dist = 0.0;
    const char * source = nullptr;

    nav_msgs::msg::Path path;
    if (getInput("path", path) && path.poses.size() >= 2) {
      // Nearest path point to the robot, so a stale leading stub doesn't skew
      // the heading or the length gate.
      size_t nearest = 0;
      double best = std::numeric_limits<double>::max();
      for (size_t i = 0; i < path.poses.size(); ++i) {
        const double d = std::hypot(
          path.poses[i].pose.position.x - rx,
          path.poses[i].pose.position.y - ry);
        if (d < best) {best = d; nearest = i;}
      }

      // Remaining path length from the nearest point to the goal.
      double remaining = 0.0;
      for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
        remaining += std::hypot(
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
      }

      // Walk forward until we've covered forward_sampling_distance, then face
      // that point — the direction the robot will actually drive.
      size_t target = path.poses.size() - 1;
      double acc = 0.0;
      for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
        acc += std::hypot(
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
        if (acc >= forward_sampling_distance) {target = i; break;}
      }

      target_x = path.poses[target].pose.position.x;
      target_y = path.poses[target].pose.position.y;
      travel_dist = remaining;
      source = "path";
    } else {
      // No usable path — fall back to the straight-line bearing to the goal so
      // the pre-rotation still fires (this is the reliability net).
      geometry_msgs::msg::PoseStamped goal;
      if (!getInput("goal", goal)) {
        gateOff("no usable path and no goal on the blackboard", true);
        return BT::NodeStatus::SUCCESS;
      }
      target_x = goal.pose.position.x;
      target_y = goal.pose.position.y;
      travel_dist = std::hypot(target_x - rx, target_y - ry);
      source = "goal-bearing (no path)";
    }

    if (travel_dist < min_path_length) {
      RCLCPP_INFO(
        node_->get_logger(),
        "ComputePathHeadingSpin: travel %.2f m < %.2f m [%s] — short move, "
        "letting MPPI reverse (no pre-rotation)", travel_dist, min_path_length, source);
      setOutput("spin_dist", 0.0);
      setOutput("do_spin", false);
      return BT::NodeStatus::SUCCESS;
    }

    // Bearing to the target, minus current heading, wrapped to (-pi, pi].
    double delta = std::atan2(target_y - ry, target_x - rx) -
      yawFromQuat(current.pose.orientation);
    while (delta > M_PI) {delta -= 2.0 * M_PI;}
    while (delta < -M_PI) {delta += 2.0 * M_PI;}

    bool do_spin = true;
    const char * reason = "facing travel direction so MPPI drives forward";
    if (std::abs(delta) < min_spin) {
      do_spin = false;
      reason = "already aligned (< min_spin)";
    } else if (std::abs(delta) > max_spin) {
      do_spin = false;
      reason = "correction exceeds max_spin";
    } else if (check_rotation_clearance &&
      !canRotateInPlace(rotation_clearance_radius, lethal_cost_threshold))
    {
      // Boxed in at the start: a Spin here would collide and stall. Skip it and
      // let MPPI drive forward out of the gap.
      do_spin = false;
      reason = "boxed in — no room to rotate, letting MPPI drive out forward";
    }

    setOutput("do_spin", do_spin);
    setOutput("spin_dist", do_spin ? delta : 0.0);
    RCLCPP_INFO(
      node_->get_logger(),
      "ComputePathHeadingSpin: %s %.3f rad (travel %.2f m) [%s] — %s",
      do_spin ? "spinning" : "not spinning", delta, travel_dist, source, reason);
    return BT::NodeStatus::SUCCESS;
  }

private:
  // True if the robot can rotate in place — no obstacle on the LOCAL costmap
  // within `clearance_radius` of the base centre. Conservative: any lethal cell
  // inside the rotation envelope (footprint corners sweep a disk of the
  // circumscribed radius) blocks the spin. Fails OPEN (returns true) when the
  // costmap or the robot pose in its frame is unavailable, so a transient hiccup
  // never silently suppresses the pre-rotation.
  bool canRotateInPlace(double clearance_radius, int lethal_threshold)
  {
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    try {
      costmap = local_costmap_sub_->getCostmap();
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ComputePathHeadingSpin: local costmap unavailable (%s) — assuming "
        "rotation is clear", e.what());
      return true;
    }

    // The local costmap is in its own (rolling, odom) frame — get the robot pose
    // there, not in the map frame, before indexing into it.
    const std::string costmap_frame = local_costmap_sub_->getFrameID();
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(
        pose, *tf_, costmap_frame, robot_base_frame_, transform_tolerance_))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "ComputePathHeadingSpin: pose in '%s' unavailable — assuming rotation "
        "is clear", costmap_frame.c_str());
      return true;
    }

    const double cx = pose.pose.position.x;
    const double cy = pose.pose.position.y;
    const double res = std::max(costmap->getResolution(), 0.01);
    const double r2 = clearance_radius * clearance_radius;

    for (double dy = -clearance_radius; dy <= clearance_radius; dy += res) {
      for (double dx = -clearance_radius; dx <= clearance_radius; dx += res) {
        if (dx * dx + dy * dy > r2) {continue;}  // outside the rotation disk
        unsigned int mx = 0, my = 0;
        if (!costmap->worldToMap(cx + dx, cy + dy, mx, my)) {continue;}  // off map
        if (static_cast<int>(costmap->getCost(mx, my)) >= lethal_threshold) {
          RCLCPP_INFO(
            node_->get_logger(),
            "ComputePathHeadingSpin: obstacle within %.2f m of base — boxed in",
            clearance_radius);
          return false;
        }
      }
    }
    return true;
  }

  void gateOff(const char * why, bool warn)
  {
    setOutput("spin_dist", 0.0);
    setOutput("do_spin", false);
    if (warn) {
      RCLCPP_WARN(node_->get_logger(), "ComputePathHeadingSpin: %s — no pre-rotation", why);
    } else {
      RCLCPP_INFO(node_->get_logger(), "ComputePathHeadingSpin: %s — no pre-rotation", why);
    }
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> local_costmap_sub_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace robot_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_navigation::ComputePathHeadingSpin>("ComputePathHeadingSpin");
}
