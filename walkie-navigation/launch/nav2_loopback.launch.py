#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_bringup.keepout_zone_launch import get_lifecycle_nodes as get_keepout_zone_nodes
from nav2_bringup.navigation_launch import get_lifecycle_nodes as get_navigation_nodes
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory("robot_navigation")
    bringup_pkg = get_package_share_directory("robot_bringup")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
    description_pkg = get_package_share_directory("walkie_description")

    default_map = os.path.join(
        nav_pkg,
        "map",
        "08_23102025_SimWalkie_awssmallhouse_slam_toolbox",
        "map.yaml",
    )
    default_nav2_config = os.path.join(
        nav_pkg,
        "config",
        "nav2",
        "nav2_mppi_walkie_params.yaml",
    )
    rviz_config = os.path.join(nav2_bringup_pkg, "rviz", "nav2_default_view.rviz")

    map_yaml_file = LaunchConfiguration("map")
    nav2_config = LaunchConfiguration("nav2_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_keepout_zones = LaunchConfiguration("use_keepout_zones")
    keepout_mask_file = LaunchConfiguration("keepout_mask")

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
    declare_use_keepout_zones = DeclareLaunchArgument(
        "use_keepout_zones",
        default_value="false",
        description="Whether to enable keepout (no-go) zone filters",
    )
    declare_keepout_mask = DeclareLaunchArgument(
        "keepout_mask",
        default_value="",
        description="Full path to keepout mask YAML file to load",
    )

    walkie_urdf = os.path.join(description_pkg, "robots", "gz_walkie.urdf.xacro")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "robot_state_publisher.launch.py")
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

    # RewrittenYaml for Loopback Simulation:
    # Forces sim time, disables 3D sensor layers, routes odom_topic to /odom
    nav2_rewritted_params = RewrittenYaml(
        source_file=nav2_config,
        root_key="",
        param_rewrites={
            "use_sim_time": "true",
            "local_costmap.local_costmap.ros__parameters.stvl_layer.enabled": "false",
            "collision_monitor.ros__parameters.pointcloud.enabled": "false",
            "collision_monitor.ros__parameters.VelocityPolygonStop.enabled": "false",
            "controller_server.ros__parameters.odom_topic": "/odom",
            "velocity_smoother.ros__parameters.odom_topic": "/odom",
            "bt_navigator.ros__parameters.odom_topic": "/odom",
        },
        convert_types=True,
    )

    # loopback_simulator is a Nav2 lifecycle node (C++ since the navigation2
    # bump). No lifecycle manager manages it, so it configures and activates
    # itself (autostart_node); until then it publishes no TF, clock or scan.
    # It needs use_sim_time like upstream's loopback_simulation.launch.py:
    # otherwise TF is stamped with wall time and the scan with sim time, and
    # collision_monitor cannot transform the scan.
    loopback_simulator = Node(
        package="nav2_loopback_sim",
        executable="loopback_simulator",
        name="loopback_simulator",
        output="screen",
        parameters=[
            nav2_rewritted_params,
            {
                "autostart_node": True,
                "use_sim_time": True,
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
            nav2_rewritted_params,
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

    # Keepout (no-go) zone servers (launches mask server and filter info server when enabled)
    keepout_zone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "keepout_zone_launch.py")
        ),
        condition=IfCondition(use_keepout_zones),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_rewritted_params,
            "keepout_mask": keepout_mask_file,
            "use_keepout_zones": use_keepout_zones,
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_rewritted_params,
            "use_keepout_zones": use_keepout_zones,
        }.items(),
    )

    # nav2_bringup's navigation_launch.py no longer starts a lifecycle manager
    # (only its bringup_launch.py does), so manage its nodes here. The name
    # matches nav2_bringup's manager, which the RViz Nav2 panel talks to.
    def launch_lifecycle_manager(context):
        lifecycle_nodes = list(get_navigation_nodes(context))
        if use_keepout_zones.perform(context).lower() == "true":
            lifecycle_nodes.extend(get_keepout_zone_nodes(context))

        return [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_nav2",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": lifecycle_nodes},
                ],
            )
        ]

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
    ld.add_action(declare_use_keepout_zones)
    ld.add_action(declare_keepout_mask)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(loopback_simulator)
    ld.add_action(map_server)
    ld.add_action(map_lifecycle_manager)
    ld.add_action(keepout_zone_launch)
    ld.add_action(nav2_navigation)
    ld.add_action(OpaqueFunction(function=launch_lifecycle_manager))
    # ld.add_action(nav_commander)
    ld.add_action(rviz)

    return ld
