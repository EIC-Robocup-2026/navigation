from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import sys

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config','slam','slam_toolbox_params.yaml'
    )

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        # pass the params file as a launch argument (common name is 'params_file';
        launch_arguments={'params_file': config_file_path}.items()
    )

    return LaunchDescription([
        slam_toolbox_launch
    ])
