import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the directory of the nav2_bringup package
    bringup_dir = get_package_share_directory("nav2_bringup")
    # This package's own launch dir — sources a forked navigation_launch.py that
    # fixes a cmd_vel remap bug in the vendored nav2_bringup (controller_server's
    # and velocity_smoother's cmd_vel both got remapped straight to cmd_vel_nav,
    # bypassing velocity_smoother and collision_monitor entirely).
    walkie_nav_launch_dir = os.path.dirname(os.path.realpath(__file__))
    default_nav2_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "nav2",
        "nav2_mppi_walkie_real_param.yaml",
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

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    nav2_config = LaunchConfiguration("nav2_config")
    keepout_mask_file = LaunchConfiguration("keepout_mask")
    use_keepout_zones = LaunchConfiguration("use_keepout_zones")

    default_map_dir = (
        get_package_share_directory("robot_navigation")
        + "/map/28_15072026_sasinHall"
    )
    default_map = default_map_dir + "/map.yaml"
    default_keepout_mask = default_map_dir + "/keepout.yaml"

    # Lifecycle nodes managed by the keepout lifecycle manager.
    keepout_lifecycle_nodes = [
        "keepout_filter_mask_server",
        "keepout_costmap_filter_info_server",
    ]

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to the map YAML file"
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

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        "use_keepout_zones",
        default_value="true",
        description="Whether to launch the keepout (no-go) zone filter servers",
    )

    declare_keepout_mask_cmd = DeclareLaunchArgument(
        "keepout_mask",
        default_value=default_keepout_mask,
        description=(
            "Full path to the keepout mask YAML. Defaults to the mask next to the "
            "default map; if you override 'map', override this to the matching "
            "map_keepout.yaml in the same map directory."
        ),
    )

    # Include localization launch file (handles AMCL and Map Server)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml_file,
            "params_file": nav2_config,
        }.items(),
    )

    # Include navigation launch file (handles BT Navigator, Controller, Planner, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(walkie_nav_launch_dir, "navigation_launch.py")
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

    # --- Keepout (no-go) zone filter servers ---
    # Publishes the keepout mask + filter info consumed by the KeepoutFilter
    # already configured in the local/global costmaps (filter_info_topic:
    # "keepout_costmap_filter_info"). Param sections (topic names, type, base,
    # multiplier) live in nav2_config; yaml_filename is overridden here.
    #
    # nav2_config is fed through RewrittenYaml because the raw file uses YAML
    # anchors/aliases (&velocity_value_p, etc.) that the rcl param parser
    # rejects ("Will not support aliasing"). RewrittenYaml re-serializes the
    # file in Python, resolving anchors to plain values, so rcl can load it.
    configured_keepout_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_config,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    keepout_filter_mask_server = Node(
        condition=IfCondition(use_keepout_zones),
        package="nav2_map_server",
        executable="map_server",
        name="keepout_filter_mask_server",
        output="screen",
        parameters=[
            configured_keepout_params,
            {"use_sim_time": use_sim_time, "yaml_filename": keepout_mask_file},
        ],
    )

    keepout_costmap_filter_info_server = Node(
        condition=IfCondition(use_keepout_zones),
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="keepout_costmap_filter_info_server",
        output="screen",
        parameters=[
            configured_keepout_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager_keepout = Node(
        condition=IfCondition(use_keepout_zones),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": keepout_lifecycle_nodes,
            }
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_config_cmd)
    ld.add_action(declare_nav_debug_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_keepout_mask_cmd)

    # Add the separated launch files
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    ld.add_action(head_tilt_near_goal)
    ld.add_action(nav_commander)

    # Keepout zone filter servers
    ld.add_action(keepout_filter_mask_server)
    ld.add_action(keepout_costmap_filter_info_server)
    ld.add_action(lifecycle_manager_keepout)

    return ld
