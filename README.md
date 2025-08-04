# MAX ROS2 Robot - Executable Files Documentation

This repository contains a ROS2-based robot control system with computer vision capabilities. Below is a comprehensive guide to all executable files in the project.

## Main Executable Scripts

### Core Robot Control
- **`robot_controller.py`** - Main robot controller with IMU integration, motor control, and path following capabilities
- **`path_planner.py`** - Path planning algorithm implementation with obstacle avoidance using PRM (Probabilistic Roadmap)

### Computer Vision
- **`camera_calibration.py`** - Camera calibration utility for setting up computer vision pipeline
- **`camera_ws.py`** - Camera workspace processing for object detection and tracking
- **`shape_detect.py`** - Shape detection and contour analysis for object recognition

## ROS2 Launch Files

### System Launch
- **`src/roomba_bringup/launch/roomba.launch.py`** - Main robot system launcher
- **`src/roomba_bringup/launch/roomba_ros2_control.launch.py`** - ROS2 control system launcher

### Visualization and Monitoring
- **`src/roomba_description/launch/display.launch.py`** - Robot visualization launcher
- **`src/roomba_description/launch/robot_web.launch.py`** - Web-based robot interface
- **`src/roomba_description/launch/robot_state.launch.py`** - Robot state publisher

## ROS2 Node Files

### Core Navigation Nodes
- **`src/roomba_bringup/roomba_bringup/path_planner_node.py`** - Path planning ROS2 node
- **`src/roomba_bringup/roomba_bringup/localization_node.py`** - Robot localization node
- **`src/roomba_bringup/roomba_bringup/odometry_node.py`** - Wheel odometry processing
- **`src/roomba_bringup/roomba_bringup/simple_velocity_controller_node.py`** - Velocity control node

### Sensor Nodes
- **`src/roomba_bringup/roomba_bringup/imu_node.py`** - IMU sensor data processing
- **`src/roomba_bringup/roomba_bringup/camera_node.py`** - Camera data acquisition
- **`src/roomba_bringup/roomba_bringup/fake_camera_node.py`** - Simulated camera for testing
- **`src/roomba_bringup/roomba_bringup/encoder_val_node.py`** - Motor encoder value processing

### Mapping and Detection
- **`src/roomba_bringup/roomba_bringup/simple_mapper_node.py`** - Environment mapping node
- **`src/roomba_bringup/roomba_bringup/object_detect_node.py`** - Object detection and recognition

### Hardware Interface
- **`src/roomba_bringup/roomba_bringup/motor_personality_node.py`** - Motor characteristic management

## Usage Instructions

### Running Main Scripts
```bash
# Main robot controller (requires hardware)
python3 robot_controller.py

# Path planning visualization
python3 path_planner.py

# Camera calibration
python3 camera_calibration.py
```

### Launching ROS2 System
```bash
# Launch complete robot system
ros2 launch roomba_bringup roomba.launch.py

# Launch with ROS2 control
ros2 launch roomba_bringup roomba_ros2_control.launch.py

# Visualization only
ros2 launch roomba_description display.launch.py
```

### Running Individual Nodes
```bash
# Navigation nodes
ros2 run roomba_bringup path_planner_node
ros2 run roomba_bringup localization_node
ros2 run roomba_bringup odometry_node

# Sensor nodes
ros2 run roomba_bringup camera_node
ros2 run roomba_bringup imu_node

# Detection and mapping
ros2 run roomba_bringup object_detect_node
ros2 run roomba_bringup simple_mapper_node
```

## Dependencies

- ROS2 (tested with Humble/Foxy)
- OpenCV for computer vision
- NumPy for numerical computations
- Matplotlib for visualization
- Adafruit CircuitPython libraries for IMU
- RPi.GPIO for Raspberry Pi GPIO control

## Hardware Requirements

- Raspberry Pi (for GPIO control)
- BNO055 IMU sensor
- Camera module
- Motor drivers and encoders
- Serial communication interface

## File Structure

```
MAX_ROS2/
├── robot_controller.py          # Main executable
├── path_planner.py             # Path planning executable
├── camera_calibration.py       # Camera calibration executable
├── camera_ws.py               # Camera workspace processing
├── shape_detect.py            # Shape detection utility
├── src/
│   ├── roomba_bringup/
│   │   ├── launch/            # ROS2 launch files
│   │   └── roomba_bringup/    # ROS2 node implementations
│   └── roomba_description/
│       └── launch/            # Visualization launch files
├── maps/                      # Map data files
├── calibration_image.jpg      # Camera calibration image
├── floor.jpg                 # Floor reference image
├── homography_matrix.npy     # Camera transformation matrix
├── motor_characterization.json # Motor parameters
├── simple_map.json          # Environment map data
└── various output images/    # Generated visualization files
```

This system provides a complete autonomous robot platform with navigation, mapping, and computer vision capabilities suitable for indoor environments.