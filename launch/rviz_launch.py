import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():
    my_package_dir = get_package_share_directory('prac_robot')
    return launch.LaunchDescription([
        Node(package='rviz2',
             executable='rviz2'
            )
    ])