#include <memory>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Custom Service Header (Make sure you generated this!)
#include "robot_navigation/srv/move_robot.hpp" 

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

using std::placeholders::_1;
using std::placeholders::_2;

class SmartServiceNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    SmartServiceNode() : Node("smart_service_node")
    {
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            map_qos, 
            std::bind(&SmartServiceNode::mapCallback, this, _1));
        
        srv_server_ = this->create_service<robot_navigation::srv::MoveRobot>(
            "move_robot",
            std::bind(&SmartServiceNode::service_callback, this, _1, _2)
        );

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), ">>> Smart Service Node Ready!");
        RCLCPP_INFO(this->get_logger(), "    Command: 'ros2 service call /move_robot ...'");
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Service<robot_navigation::srv::MoveRobot>::SharedPtr srv_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    void service_callback(
        const std::shared_ptr<robot_navigation::srv::MoveRobot::Request> request,
        std::shared_ptr<robot_navigation::srv::MoveRobot::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "--- Service Request: (%.2f, %.2f) ---", request->x, request->y);

        if (!current_map_) {
            response->success = false;
            response->message = "Failed: No Costmap received yet.";
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: Map not ready.");
            return;
        }

        double final_x = request->x;
        double final_y = request->y;
        bool adjusted = false;

        if (!isPointFree(final_x, final_y, true)) {
            RCLCPP_WARN(this->get_logger(), "Target OBSTRUCTED. Searching for safe spot...");
            
            auto safe_pose = findNearestValidPose(final_x, final_y, "map");
            if (safe_pose) {
                final_x = safe_pose->pose.position.x;
                final_y = safe_pose->pose.position.y;
                adjusted = true;
                RCLCPP_INFO(this->get_logger(), ">>> Fixed: New target is (%.2f, %.2f)", final_x, final_y);
            } else {
                response->success = false;
                response->message = "Failed: Goal is blocked and no safe point found.";
                RCLCPP_ERROR(this->get_logger(), "FAILED: Could not find safe point.");
                return;
            }
        }

        auto p = std::make_shared<geometry_msgs::msg::PoseStamped>();
        p->header.frame_id = "map";
        p->header.stamp = this->now();
        p->pose.position.x = final_x;
        p->pose.position.y = final_y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, request->theta);
        p->pose.orientation = tf2::toMsg(q);

        sendNavGoal(*p);

        response->success = true;
        if (adjusted) {
            response->message = "Goal adjusted to nearest safe point.";
        } else {
            response->message = "Goal accepted.";
        }
    }

    void sendNavGoal(const geometry_msgs::msg::PoseStamped& pose)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server offline.");
            return;
        }
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;
        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        nav_client_->async_send_goal(goal_msg, opts);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
    }

    bool isPointFree(double wx, double wy, bool verbose = false) {
        if (!current_map_) return false;
        unsigned int mx, my;
        if (!worldToMap(wx, wy, mx, my)) return false; 
        size_t index = (size_t)my * current_map_->info.width + (size_t)mx;
        if (index >= current_map_->data.size()) return false; 
        int8_t cost = current_map_->data[index];
        if (verbose) RCLCPP_INFO(this->get_logger(), "Cost Check: %d", cost);
        if (cost == -1) return false;
        return (cost < 50);
    }

    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
        double origin_x = current_map_->info.origin.position.x;
        double origin_y = current_map_->info.origin.position.y;
        double res = current_map_->info.resolution;
        if (wx < origin_x || wy < origin_y) return false;
        mx = (unsigned int)((wx - origin_x) / res);
        my = (unsigned int)((wy - origin_y) / res);
        return (mx < current_map_->info.width && my < current_map_->info.height);
    }

    std::shared_ptr<geometry_msgs::msg::PoseStamped> findNearestValidPose(double tx, double ty, std::string frame_id) {
        double max_radius = 2.0; 
        double step_r = 0.05;    
        double step_th = 0.1;    

        for (double r = step_r; r <= max_radius; r += step_r) {
            for (double theta = 0.0; theta < 2.0 * M_PI; theta += step_th) {
                double check_x = tx + r * cos(theta);
                double check_y = ty + r * sin(theta);
                if (isPointFree(check_x, check_y, false)) {
                    auto p = std::make_shared<geometry_msgs::msg::PoseStamped>();
                    p->header.stamp = this->now();
                    p->header.frame_id = frame_id;
                    p->pose.position.x = check_x;
                    p->pose.position.y = check_y;
                    return p;
                }
            }
        }
        return nullptr;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartServiceNode>());
    rclcpp::shutdown();
    return 0;
}