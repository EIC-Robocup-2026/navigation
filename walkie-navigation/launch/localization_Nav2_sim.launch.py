import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the directory of the nav2_bringup package
    bringup_dir = get_package_share_directory("nav2_bringup")
    default_nav2_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "nav2",
        "nav2_mppi_walkie_params.yaml",
    )
    bt_xml_path = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "nav2",
        "behavior_tree",
        "nav_topose_no_spin_backup_bt.xml",
    )
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    nav2_config = LaunchConfiguration("nav2_config")
    use_pointcloud = LaunchConfiguration("use_pointcloud")

    default_map = os.path.join(
        get_package_share_directory("robot_navigation"),
        "map",
        "08_23102025_SimWalkie_awssmallhouse_slam_toolbox",
        "map.yaml",
    )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to the map YAML file"
    )

    declare_nav2_config_cmd = DeclareLaunchArgument(
        "nav2_config",
        default_value=default_nav2_config,
        description="Full path to the nav2 config YAML file",
    )

    declare_use_pointcloud_cmd = DeclareLaunchArgument(
        "use_pointcloud",
        default_value="false",
        description="Whether to enable 3D pointcloud layers in simulation",
    )

    declare_nav_debug_cmd = DeclareLaunchArgument(
        "nav_debug",
        default_value="false",
        description="Publish nav_commander debug markers (ROI cells, PCA, approach angle)",
    )

    # Simulation parameter substitutions via RewrittenYaml
    nav2_rewritted_params = RewrittenYaml(
        source_file=nav2_config,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "local_costmap.local_costmap.ros__parameters.stvl_layer.enabled": use_pointcloud,
            "collision_monitor.ros__parameters.pointcloud.enabled": use_pointcloud,
        },
        convert_types=True,
    )

    # Include navigation launch file.
    # Nav2 nodes publish "cmd_vel"; remap it to cmd_vel_controller so commands
    # pass velocity_smoother and collision_monitor before reaching twist_mux on
    # cmd_vel_nav. SetRemap rules come before nav2_bringup's own
    # cmd_vel -> cmd_vel_nav remaps, and the first matching rule wins.
    # use_composition is off: the Jazzy component_container ignores
    # --isolated, and bringup can deadlock while loading the lifecycle manager.
    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src="cmd_vel", dst="cmd_vel_controller"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml_file,
                    "params_file": nav2_rewritted_params,
                    "default_nav_to_pose_bt_xml": bt_xml_path,
                    "use_composition": "False",
                    "use_keepout_zones": "false",
                }.items(),
            ),
        ]
    )
    # Object approach-pose action server (/navigate_to_object -> Nav2)
    walkie_nav_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "walkie_nav.yaml",
    )
    nav_commander = Node(
        package="robot_navigation",
        executable="nav_commander.py",
        name="nav_commander",
        output="screen",
        parameters=[
            walkie_nav_config,
            {"use_sim_time": use_sim_time,
             "debug": LaunchConfiguration("nav_debug")},
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_config_cmd)
    ld.add_action(declare_use_pointcloud_cmd)
    ld.add_action(declare_nav_debug_cmd)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(nav_commander)

    return ld
