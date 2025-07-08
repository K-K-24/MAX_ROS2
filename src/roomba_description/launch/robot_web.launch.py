import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the package
    pkg_path = os.path.join(get_package_share_directory('roomba_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'roomba.urdf.xacro')
    
    # Create launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # ROS API Node (Required for Foxglove)
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen',
            parameters=[{
                'topics_glob_timeout': 0.5,
                'services_glob_timeout': 0.5,
                'params_glob_timeout': 0.5
            }]
        ),
        
        # Rosbridge WebSocket Server
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            arguments = ['--host', '0.0.0.0', '--port', '9090']
        )
    ])
