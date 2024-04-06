from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  
def generate_launch_description():
    parameters=[]
    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/model/leorover/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]


    bridge_remappings = [
        ('/clock', '/leo/clock'),
        ('/scan', '/leo/scan'),
        ('/model/leorover/tf','/leo/tf'), # Renamed as otherwise, the leorover/odom frame will interfere with SLAM


    ]

    return LaunchDescription([

    # Nodes to launch
    Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=bridge_arguments,
        remappings=bridge_remappings),
    ])