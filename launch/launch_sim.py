
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_name = 'prac_robot'
    pkg_path = get_package_share_directory(pkg_name)

    ## xarco to URDF
    print('aaa1\n')
    xacro_path = os.path.join(pkg_path, 'description', 'example_robot.urdf.xacro')
    world_path = os.path.join(pkg_path, 'description', 'messy_world.sdf')
    print('bbb4\n')
    robot_description_config = xacro.process_file(xacro_path, mappings={"use_ros2_control": use_ros2_control, "sim_mode": use_sim_time}).toxml()
    #robot_description_config = Command(['xacro', xacro_path, 'use_ros2_control:=', use_ros2_control,'sim_mode:=', use_sim_time])
    #robot_description_config = 'diff_drive_robot.urdf'
    print('ddd1\n')
    ## launch robot state publisher

    ### check this
    #https://answers.ros.org/question/404549/unable-to-parse-urdf-using-command-line-but-can-do-it-in-launch-file/
    
    
    
    params = {'robot_description':robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable= 'robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')



    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_path}.items(),
    )
    
    gz_create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'messy_world',
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description', #use topic entry##
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen',
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'),
        DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'),

        node_robot_state_publisher,
        gz_sim,
        gz_create
    ])