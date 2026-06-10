cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
# For the ComputeGoalHeadingSpin behavior-tree plugin
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)


# Generate the Services and Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobot.srv"
  "action/NavigateToObject.action"
  DEPENDENCIES geometry_msgs action_msgs
)

add_executable(Sgoal_Service_node robot_navigation/SgoalService.cpp) 

ament_target_dependencies(Sgoal_Service_node
  rclcpp
  std_msgs
  rclcpp_action
  nav2_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

rosidl_target_interfaces(Sgoal_Service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  Sgoal_Service_node
  DESTINATION lib/${PROJECT_NAME}
)

# ComputeGoalHeadingSpin BT plugin — loaded by bt_navigator via plugin_lib_names.
# Installed to lib/ so the BT factory finds it on the library path.
add_library(compute_goal_heading_spin_bt_node SHARED
  robot_navigation/compute_goal_heading_spin.cpp)
target_compile_definitions(compute_goal_heading_spin_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_goal_heading_spin_bt_node
  behaviortree_cpp
  geometry_msgs
  nav2_util
  nav2_ros_common
  rclcpp
  tf2_ros
)
install(TARGETS compute_goal_heading_spin_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# ComputeGoalBearingSpin BT plugin — spins the base to face the goal *position*
# (so the tilt-only head can mark the obstacle) before the STVL freeze.
add_library(compute_goal_bearing_spin_bt_node SHARED
  robot_navigation/compute_goal_bearing_spin.cpp)
target_compile_definitions(compute_goal_bearing_spin_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_goal_bearing_spin_bt_node
  behaviortree_cpp
  geometry_msgs
  nav2_util
  nav2_ros_common
  rclcpp
  tf2_ros
)
install(TARGETS compute_goal_bearing_spin_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python robot_navigation
install(PROGRAMS
  robot_navigation/human_follower_node.py
  robot_navigation/accel_stamp_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  robot_navigation/current_pose_publisher.py
  robot_navigation/pointcloud_self_filter_node.py
  robot_navigation/stvl_decay_manager_node.py
  robot_navigation/head_tilt_near_goal_node.py
  DESTINATION lib/${PROJECT_NAME}
)
# nav_commander imports approach_pose/costmap_reader as sibling modules, so
# all three must land in the same lib/${PROJECT_NAME} directory.
install(PROGRAMS
  robot_navigation/nav_commander.py
  robot_navigation/approach_pose.py
  robot_navigation/costmap_reader.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY
    launch
    config
    map
    srv
    action
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
