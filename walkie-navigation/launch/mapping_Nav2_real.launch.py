import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of the nav2_bringup package
    bringup_dir = get_package_share_directory("nav2_bringup")
    default_nav2_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "nav2",
        "nav2_mppi_walkie_real_restaurant_param.yaml",
    )
    bt_xml_path = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "nav2",
        "behavior_tree",
        "nav_to_pose_real_recovery_bt.xml",
    )
    walkie_nav_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "walkie_nav.yaml",
    )
    # SLAM Toolbox parameters (online sync mapping mode)
    slam_params_file = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "slam",
        "slam_toolbox_params.yaml",
    )

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_config = LaunchConfiguration("nav2_config")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_nav2_config_cmd = DeclareLaunchArgument(
        "nav2_config",
        default_value=default_nav2_config,
        description="Full path to the nav2 config YAML file",
    )

    declare_nav_debug_cmd = DeclareLaunchArgument(
        "nav_debug",
        default_value="false",
        description="Publish nav_commander debug markers (ROI cells, PCA, approach angle)",
    )

    # Include SLAM Toolbox (online sync) instead of localization. SLAM provides the
    # map->odom transform and builds/publishes the map live, so no AMCL or Map Server
    # (and no pre-saved map) is needed in this mode.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_sync_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # Include navigation launch file (handles BT Navigator, Controller, Planner, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_config,
            "default_nav_to_pose_bt_xml": bt_xml_path,
        }.items(),
    )

    # Tilt the ZED head down near the goal so tabletop obstacles stay in the
    # camera FOV at close range (live marking) instead of dropping into the blind
    # spot. Replaces the old STVL freeze + near-goal stop.
    head_tilt_near_goal = Node(
        package="robot_navigation",
        executable="head_tilt_near_goal_node.py",
        name="head_tilt_near_goal",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "head_command_topic": "/head_servo_controller/commands",
                "near_distance": 1.5,
                "far_distance": 1.7,
                "near_ignore_distance": 0.3,
                "default_angle": 0.25,
                "down_angle": 0.785,
                # Start owning the head; an external camera controller can call
                # the SetBool service '/head_tilt_near_goal/enable' to take over.
                "start_enabled": True,
            }
        ],
    )

    # Object approach-pose action server (/navigate_to_object -> Nav2)
    nav_commander = Node(
        package="robot_navigation",
        executable="nav_commander.py",
        name="nav_commander",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("robot_navigation")
                         , "config", "walkie_nav.yaml"),
            {"use_sim_time": use_sim_time,
             "debug": LaunchConfiguration("nav_debug")},
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav2_config_cmd)
    ld.add_action(declare_nav_debug_cmd)

    # Add the separated launch files
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)
    ld.add_action(head_tilt_near_goal)
    ld.add_action(nav_commander)

    return ld
