#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_navigation
{

class AdjustGoalToCostmap : public BT::SyncActionNode
{
public:
  AdjustGoalToCostmap(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
      },
      map_qos,
      callback_group_);
  }

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal pose to check and adjust"),
      BT::InputPort<double>("cost_threshold", 50.0, "Max cost to consider free (0-100)"),
      BT::InputPort<double>("max_search_radius", 2.0, "Spiral search radius in meters"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Adjusted goal pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();

    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);

    double cost_threshold, max_radius;
    getInput("cost_threshold", cost_threshold);
    getInput("max_search_radius", max_radius);

    if (!current_map_) {
      RCLCPP_WARN(node_->get_logger(),
        "[AdjustGoalToCostmap] No costmap yet — passing goal through.");
      setOutput("output_goal", goal);
      return BT::NodeStatus::SUCCESS;
    }

    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    if (isPointFree(gx, gy, cost_threshold)) {
      setOutput("output_goal", goal);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(),
      "[AdjustGoalToCostmap] Goal (%.2f, %.2f) blocked — searching for free cell...", gx, gy);

    const double step_r = 0.05;
    const double step_th = 0.1;

    for (double r = step_r; r <= max_radius; r += step_r) {
      for (double th = 0.0; th < 2.0 * M_PI; th += step_th) {
        double cx = gx + r * std::cos(th);
        double cy = gy + r * std::sin(th);
        if (isPointFree(cx, cy, cost_threshold)) {
          geometry_msgs::msg::PoseStamped adjusted = goal;
          adjusted.pose.position.x = cx;
          adjusted.pose.position.y = cy;

          // Orient adjusted goal toward the originally requested point
          tf2::Quaternion q;
          q.setRPY(0, 0, std::atan2(gy - cy, gx - cx));
          adjusted.pose.orientation = tf2::toMsg(q);

          RCLCPP_INFO(node_->get_logger(),
            "[AdjustGoalToCostmap] Adjusted to (%.2f, %.2f), facing original goal.", cx, cy);
          setOutput("output_goal", adjusted);
          return BT::NodeStatus::SUCCESS;
        }
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
      "[AdjustGoalToCostmap] No free cell within %.1f m — rejecting goal.", max_radius);
    return BT::NodeStatus::FAILURE;
  }

private:
  nav2::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  bool isPointFree(double wx, double wy, double threshold)
  {
    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) {return false;}
    size_t idx = static_cast<size_t>(my) * current_map_->info.width + static_cast<size_t>(mx);
    if (idx >= current_map_->data.size()) {return false;}
    int8_t cost = current_map_->data[idx];
    if (cost == -1) {return false;}
    return cost < static_cast<int8_t>(threshold);
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
  {
    double ox = current_map_->info.origin.position.x;
    double oy = current_map_->info.origin.position.y;
    double res = current_map_->info.resolution;
    if (wx < ox || wy < oy) {return false;}
    mx = static_cast<unsigned int>((wx - ox) / res);
    my = static_cast<unsigned int>((wy - oy) / res);
    return mx < current_map_->info.width && my < current_map_->info.height;
  }
};

}  // namespace robot_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_navigation::AdjustGoalToCostmap>("AdjustGoalToCostmap");
}
