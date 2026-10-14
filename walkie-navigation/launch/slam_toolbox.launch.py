import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'slam', 'slam_toolbox_params.yaml'
    )

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_sync_launch.py'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo / O3DE) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_config_path,
        description='Full path to the slam_toolbox parameters YAML file'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        slam_toolbox_launch,
    ])
