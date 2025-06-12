import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    roomba_description_dir = get_package_share_directory('roomba_description')
    roomba_bringup_dir = get_package_share_directory('roomba_bringup')
    
    # Paths
    urdf_file = os.path.join(roomba_description_dir, 'urdf', 'roomba.urdf.xacro')
    controller_config_file = os.path.join(roomba_bringup_dir, 'config', 'roomba_controllers.yaml')
    
    # Check if files exist
    print(f"URDF file exists: {os.path.exists(urdf_file)}")
    print(f"Controller config exists: {os.path.exists(controller_config_file)}")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Your existing sensor nodes
        Node(
            package='roomba_bringup',
            executable='sensor_reader',
            name='sensor_reader_node',
            output='screen',
        ),
        
        Node(
            package='roomba_bringup',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
        
        # Simple velocity controller (receives commands from ros2_control)
        Node(
            package='roomba_bringup',
            executable='simple_velocity_controller',
            name='simple_velocity_controller_node',
            output='screen',
        ),

                # Simple velocity controller (receives commands from ros2_control)
        Node(
            package='roomba_bringup',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
        
        # ros2_control Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }, controller_config_file],
            output='screen',
        ),
        
        # Joint State Broadcaster Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        
        # Differential Drive Controller Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])