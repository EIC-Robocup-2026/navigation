// Compute the in-place rotation (relative yaw) that makes the robot face the
// goal's commanded heading.
//
// Used in the behavior tree's failure path: when a goal aborts (e.g. it was
// commanded inside a table and is unreachable), the robot should still end up
// facing the commanded orientation. This node reads the goal from the blackboard
// and the robot's current pose from TF, and outputs spin_dist = the shortest
// signed yaw delta from the current heading to the goal heading, which a
// downstream Spin behavior then executes.

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

class ComputeGoalHeadingSpin : public BT::SyncActionNode
{
public:
  ComputeGoalHeadingSpin(
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

  ComputeGoalHeadingSpin() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Goal pose whose heading the robot should face"),
      BT::OutputPort<double>(
        "spin_dist", "Relative yaw (rad) to rotate to face the goal heading"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "ComputeGoalHeadingSpin: no goal on the blackboard");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped current;
    if (!nav2_util::getCurrentPose(
        current, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
      RCLCPP_WARN(node_->get_logger(), "ComputeGoalHeadingSpin: current robot pose unavailable");
      return BT::NodeStatus::FAILURE;
    }

    // Shortest signed angular distance from current heading to goal heading,
    // wrapped to (-pi, pi]. Both poses are in global_frame.
    double delta = yawFromQuat(goal.pose.orientation) - yawFromQuat(current.pose.orientation);
    while (delta > M_PI) {delta -= 2.0 * M_PI;}
    while (delta < -M_PI) {delta += 2.0 * M_PI;}

    setOutput("spin_dist", delta);
    RCLCPP_INFO(
      node_->get_logger(),
      "ComputeGoalHeadingSpin: goal aborted; spinning %.3f rad to face goal heading", delta);
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
  factory.registerNodeType<robot_navigation::ComputeGoalHeadingSpin>("ComputeGoalHeadingSpin");
}
