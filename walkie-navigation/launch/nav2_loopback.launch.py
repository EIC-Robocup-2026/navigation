#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("robot_navigation")
    bringup_pkg = get_package_share_directory("robot_bringup")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
    description_pkg = get_package_share_directory("walkie_description")

    default_map = os.path.join(
        nav_pkg,
        "map",
        "8_23102025_SimWalkie_awssmallhouse_slam_toolbox",
        "map.yaml",
    )
    default_nav2_config = os.path.join(
        nav_pkg,
        "config",
        "nav2",
        "nav2_mppi_walkie_loopback_params.yaml",
    )
    rviz_config = os.path.join(nav2_bringup_pkg, "rviz", "nav2_default_view.rviz")

    map_yaml_file = LaunchConfiguration("map")
    nav2_config = LaunchConfiguration("nav2_config")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map YAML file",
    )
    declare_nav2_config = DeclareLaunchArgument(
        "nav2_config",
        default_value=default_nav2_config,
        description="Full path to Nav2 params YAML file",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz",
    )
    declare_nav_debug = DeclareLaunchArgument(
        "nav_debug",
        default_value="false",
        description="Publish nav_commander debug markers (ROI cells, PCA, approach angle)",
    )

    walkie_urdf = os.path.join(description_pkg, "robots", "gz_walkie.urdf.xacro")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "ros2_control": "mock",
            "use_zed": "false",
            "robot_model": walkie_urdf,
        }.items(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
    )

    loopback_simulator = Node(
        package="nav2_loopback_sim",
        executable="loopback_simulator",
        name="loopback_simulator",
        output="screen",
        parameters=[
            nav2_config,
            {
                "base_frame_id": "base_footprint",
                "scan_frame_id": "base_footprint",
                "publish_map_odom_tf": True,
                "publish_clock": True,
                "enable_stamped_cmd_vel": False,
            },
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_config,
            {"yaml_filename": map_yaml_file, "use_sim_time": True},
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_config,
        }.items(),
    )

    # Object approach-pose action server (/navigate_to_object -> Nav2).
    # No AMCL in loopback sim; the commander falls back to TF for robot pose.
    nav_commander = Node(
        package="robot_navigation",
        executable="nav_commander.py",
        name="nav_commander",
        output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "walkie_nav.yaml"),
            {"use_sim_time": True,
             "debug": LaunchConfiguration("nav_debug")},
        ],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_nav2_config)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_nav_debug)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(loopback_simulator)
    ld.add_action(map_server)
    ld.add_action(map_lifecycle_manager)
    ld.add_action(nav2_navigation)
    # ld.add_action(nav_commander)
    ld.add_action(rviz)

    return ld
