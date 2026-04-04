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


# Generate the Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs
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

# Install Python robot_navigation
install(PROGRAMS
  robot_navigation/human_follower_node.py
  robot_navigation/accel_stamp_node.py
  robot_navigation/goPose_Backend.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  robot_navigation/current_pose_publisher.py
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
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
