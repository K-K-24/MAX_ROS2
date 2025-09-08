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
    slam_config_file = os.path.join(roomba_bringup_dir,'config','slam_toolbox_config.yaml')
    
    # Check if files exist
    print(f"URDF file exists: {os.path.exists(urdf_file)}")
    print(f"Controller config exists: {os.path.exists(controller_config_file)}")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #RPLidar launch arguments
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port",default="/dev/rplidar")
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='true')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description="Specifying the channel type of lidar"
        ),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description="Specifying usb port to connected lidar"
        ),

              DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
        
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),
        
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        
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

        #RPLidar Node
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[{
                'channel_type':channel_type,
                'serial_port':serial_port,
                'serial_baudrate':serial_baudrate,
                'frame_id':frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        #SLAM TOOLBOX NODE
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_file]
        ),
        
        # Simple velocity controller (receives commands from ros2_control)
        Node(
            package='roomba_bringup',
            executable='simple_velocity_controller',
            name='simple_velocity_controller_node',
            output='screen',
        ),

               
        Node(
            package='roomba_bringup',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),


        # ros2_control Controller Manager
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[{
        #         'robot_description': robot_description,
        #         'use_sim_time': use_sim_time
        #     }, controller_config_file],
        #     output='screen',
        # ),
        
        # Joint State Broadcaster Spawner
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        #     output='screen',
        # ),
        
        # # Differential Drive Controller Spawner
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        #     output='screen',
        # ),

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