
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.actions import IncludeLaunchDescription

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    pkg_name = 'prac_robot'
    pkg_path = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # description path
    xacro_path = os.path.join(pkg_path, 'description', 'prac_robot.urdf.xacro')
    world_path = os.path.join(pkg_path, 'description', 'depot_world.sdf')


    # xarco to URDF
    robot_description_config = xacro.process_file(xacro_path, mappings={"sim_mode": use_sim_time}).toxml()

    # launch robot state publisher
    params = {'robot_description':robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable= 'robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # define robot spawn position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='2.0')
    Y_pose = LaunchConfiguration('Y_pose', default='0.0')

    # launch gazebo with environment
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_path}.items(),
    )
    
    # spawn robot from topic
    gz_create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'depot_world',
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description', #use topic entry##
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', Y_pose
        ],
        output='screen',
    )
    



    '''
    # launch joint_state_broadcaster 
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )


    # launch diff_drive_base_controller
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )
    '''

    #robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(pkg_path,'config','prac_controller.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable= 'ros2_control_node',
        parameters=[{'robot_description': robot_description_config}, controller_params_file]
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )




    # need this bridge, otherwise control 2 teleop won't work
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # lidar bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/ignLidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'

    )

    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
    keyboardControl = Node(
        package='teleop_twist_keyboard',
        executable= 'teleop_twist_keyboard',
        arguments=['cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped'],
        output='screen',
        prefix="xterm -e" # open another terminal, otherwise this node won't run
                          # requires dependency: xterm

    )

    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_path,'launch','slam_online_async_map.py'
                )])
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments={'-d': os.path.join(pkg_path, 'config', 'explore_config.rviz')}.items(),
    )


    return LaunchDescription([
        gz_sim,
        gz_create,
        clock_bridge,
        lidar_bridge,
        slam_toolbox,
        node_robot_state_publisher,
        keyboardControl,
        
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'),
        
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rviz
    ])