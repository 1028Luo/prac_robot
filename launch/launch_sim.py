
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    package_name = 'prac_robot'

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'messy_world.sdf'}.items(),
    )
    

    ## under construction
    description_path = os.path.join(get_package_share_directory(package_name),'description', 'prac_robot.urdf.xarco')
    
    
    
    
    
    
    
    gz_create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'messy_world',
            '-name', 'diff_drive_robot',
            '-file', robot_desc,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
            #use topic entry##
        ],
        output='screen',
    )
    

    ld = LaunchDescription()
    #ld.add_action(gzserver_cmd)
    ld.add_action(gz_sim)
    ld.add_action(gz_create)
    return ld