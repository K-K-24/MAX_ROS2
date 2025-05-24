

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Create launch arguments for optional components
    include_test_motors = LaunchConfiguration('include_test_motors')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'include_test_motors',
            default_value='false',
            description='Whether to include the test_motors node'
        ),
        
        # Core nodes
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
        
        Node(
            package='roomba_bringup',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
        
        Node(
            package='roomba_bringup',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
        ),
        

    ])