// Compute the in-place rotation (relative yaw) that points the robot's front
// (and the forward-mounted ZED head) at the goal *position*.
//
// Why this exists
// ---------------
// Walkie is a holonomic omnibot, so MPPI can translate straight at the goal
// without ever yawing to face it. The head only tilts (pitch), it cannot pan,
// so if the base is not facing the goal the camera never sees the tabletop
// obstacle sitting at the goal. The STVL freeze (stvl_decay_manager_node.py)
// then latches an *empty* snapshot through the close-range blind spot and the
// robot drives into the table.
//
// Used in the verify-pause before the freeze: the robot stops ~2 m out, spins
// to face the goal so the head re-marks the obstacle into the local costmap,
// holds for the confirm dwell, and only then drives the last stretch into the
// blind spot on a populated (and soon-to-be-frozen) snapshot.
//
// Unlike ComputeGoalHeadingSpin (which faces the goal's commanded ORIENTATION),
// this faces the BEARING from the robot to the goal position.

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

namespace robot_navigation
{

class ComputeGoalBearingSpin : public BT::SyncActionNode
{
public:
  ComputeGoalBearingSpin(
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
  }

  ComputeGoalBearingSpin() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Goal pose whose position the robot should face"),
      // Gating: skip the spin when it is not worth it, so short reposition moves
      // (e.g. a small backward nudge) don't make the robot whirl around.
      BT::InputPort<double>(
        "min_face_dist", 0.5,
        "Don't spin if the goal is closer than this (m) — it's a reposition, "
        "not a blind-spot approach"),
      BT::InputPort<double>(
        "min_spin", 0.15,
        "Don't spin for corrections smaller than this (rad) — already facing"),
      BT::InputPort<double>(
        "max_spin", 2.4,
        "Don't spin for corrections larger than this (rad) — a near-reversal, "
        "e.g. a backward-commanded goal"),
      BT::OutputPort<double>(
        "spin_dist", "Relative yaw (rad) to rotate to face the goal position "
        "(0 when the spin is gated off)"),
      BT::OutputPort<bool>(
        "do_spin", "False when the spin is gated off, so the BT can skip Spin"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "ComputeGoalBearingSpin: no goal on the blackboard");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped current;
    if (!nav2_util::getCurrentPose(
        current, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
      RCLCPP_WARN(node_->get_logger(), "ComputeGoalBearingSpin: current robot pose unavailable");
      return BT::NodeStatus::FAILURE;
    }

    const double dx = goal.pose.position.x - current.pose.position.x;
    const double dy = goal.pose.position.y - current.pose.position.y;

    // Bearing from the robot to the goal position, then subtract the current
    // heading to get the shortest signed in-place rotation, wrapped to (-pi, pi].
    double delta = std::atan2(dy, dx) - yawFromQuat(current.pose.orientation);
    while (delta > M_PI) {delta -= 2.0 * M_PI;}
    while (delta < -M_PI) {delta += 2.0 * M_PI;}

    double min_face_dist = 0.5, min_spin = 0.15, max_spin = 2.4;
    getInput("min_face_dist", min_face_dist);
    getInput("min_spin", min_spin);
    getInput("max_spin", max_spin);

    const double dist = std::hypot(dx, dy);
    bool do_spin = true;
    const char * reason = "facing goal so the head can mark it";
    if (dist < min_face_dist) {
      do_spin = false;
      reason = "goal within min_face_dist — short reposition, not an approach";
    } else if (std::abs(delta) < min_spin) {
      do_spin = false;
      reason = "already within min_spin of the goal bearing";
    } else if (std::abs(delta) > max_spin) {
      do_spin = false;
      reason = "would need a near-reversal (> max_spin) — e.g. a backward move";
    }

    setOutput("do_spin", do_spin);
    setOutput("spin_dist", do_spin ? delta : 0.0);
    if (do_spin) {
      RCLCPP_INFO(
        node_->get_logger(),
        "ComputeGoalBearingSpin: spinning %.3f rad — %s", delta, reason);
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "ComputeGoalBearingSpin: not spinning (delta %.3f rad, dist %.2f m) — %s",
        delta, dist, reason);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace robot_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_navigation::ComputeGoalBearingSpin>("ComputeGoalBearingSpin");
}
