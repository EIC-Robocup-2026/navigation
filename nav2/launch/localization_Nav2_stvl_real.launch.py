from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the nav2_bringup package
    bringup_dir = get_package_share_directory('nav2_bringup')
    default_nav2_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'nav2' ,
        'nav2_mppi_walkie_real_param_stvl.yaml'
    )

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav2_config = LaunchConfiguration('nav2_config')
    # rviz_config_file = get_package_share_directory('robot_navigation') + 'rviz/nav.rviz'

    default_map = get_package_share_directory('robot_navigation') + '/map/9_091125_Walkie_EIC_slam_toolbox/map.yaml'

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map YAML file'
    )

    declare_nav2_config_cmd = DeclareLaunchArgument(
        'nav2_config',
        default_value=default_nav2_config,
        description='Full path to the nav2 config YAML file'
    )

    # Include localization launch file
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file' : nav2_config,
        }.items(),
    )

    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file' : nav2_config,
        }.items()
    )

    # Include navigation launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file' : nav2_config,
        }.items()
    )
    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(package_dir, 'launch', 'rviz.launch.py')),
    #     condition=IfCondition(use_rviz),
    #     launch_arguments={'rviz_config': rviz_config_file}.items())

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_config_cmd)
    # ld.add_action(rviz_cmd)


    # Add both localization and navigation launches
    # ld.add_action(localization_launch)
    # ld.add_action(navigation_launch)
    ld.add_action(nav2_bringup_launch)

    return ld
