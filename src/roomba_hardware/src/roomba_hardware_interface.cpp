#include "roomba_hardware/roomba_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roomba_hardware
{

hardware_interface::CallbackReturn RoombaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
    "🔧 Wheel radius: %.3f m, Wheel separation: %.3f m", 
    wheel_radius_, wheel_separation_);

  // Initialize joint states
  left_wheel_pos_ = 0.0;
  left_wheel_vel_ = 0.0;
  right_wheel_pos_ = 0.0;
  right_wheel_vel_ = 0.0;
  
  // Initialize commands
  left_wheel_vel_cmd_ = 0.0;
  right_wheel_vel_cmd_ = 0.0;
  
  // Initialize encoder tracking
  left_encoder_pos_ = 0.0;
  right_encoder_pos_ = 0.0;
  last_left_encoder_ = 0.0;
  last_right_encoder_ = 0.0;
  sensor_data_received_ = false;

  // Check joint configuration
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
      "📍 Found joint: %s", joint.name.c_str());
      
    // Verify we have the expected joints
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoombaHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoombaHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoombaHardwareInterface"),
        "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create ROS2 node for communication
  node_ = std::make_shared<rclcpp::Node>("roomba_hardware_interface");
  
  // Create publisher to send commands to your simple_velocity_controller
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  // Subscribe to sensor data from your sensor_reader
  sensor_sub_ = node_->create_subscription<roomba_interfaces::msg::SensorData>(
    "/wheel_states", 10,
    std::bind(&RoombaHardwareInterface::sensor_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "✅ Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoombaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Use the joint names from the URDF info
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
      "📤 Exporting state interfaces for joint: %s", info_.joints[i].name.c_str());
      
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, 
      (i == 0) ? &left_wheel_pos_ : &right_wheel_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, 
      (i == 0) ? &left_wheel_vel_ : &right_wheel_vel_));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoombaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Use the joint names from the URDF info
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
      "📥 Exporting command interfaces for joint: %s", info_.joints[i].name.c_str());
      
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, 
      (i == 0) ? &left_wheel_vel_cmd_ : &right_wheel_vel_cmd_));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RoombaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "🚀 Activating ...please wait...");
  
  // Reset all values
  left_wheel_pos_ = 0.0;
  left_wheel_vel_ = 0.0;
  right_wheel_pos_ = 0.0;
  right_wheel_vel_ = 0.0;
  left_wheel_vel_cmd_ = 0.0;
  right_wheel_vel_cmd_ = 0.0;
  
  last_sensor_time_ = node_->get_clock()->now();

  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "✅ Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "🛑 Deactivating ...please wait...");
  
  // Send stop command
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(twist_msg);

  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "✅ Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoombaHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Process any pending ROS messages
  rclcpp::spin_some(node_);
  
  // Update wheel positions and velocities from encoder data
  if (sensor_data_received_)
  {
    // Convert encoder positions from cm to radians
    double left_pos_rad = (left_encoder_pos_ / 100.0) / wheel_radius_;  // cm to m to rad
    double right_pos_rad = (right_encoder_pos_ / 100.0) / wheel_radius_;
    
    // Calculate velocities
    double dt = period.seconds();
    if (dt > 0.0)
    {
      left_wheel_vel_ = (left_pos_rad - left_wheel_pos_) / dt;
      right_wheel_vel_ = (right_pos_rad - right_wheel_pos_) / dt;
    }
    
    // Update positions
    left_wheel_pos_ = left_pos_rad;
    right_wheel_pos_ = right_pos_rad;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoombaHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 🔍 DEBUG: Show what commands we received
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
    "📨 WRITE() called - Wheel commands received: L=%.3f, R=%.3f rad/s", 
    left_wheel_vel_cmd_, right_wheel_vel_cmd_);

  // Convert wheel velocities to robot linear/angular velocities
  // Note: The commands come in as rad/s for each wheel
  double linear_vel = (left_wheel_vel_cmd_ + right_wheel_vel_cmd_) * wheel_radius_ / 2.0;
  double angular_vel = (right_wheel_vel_cmd_ - left_wheel_vel_cmd_) * wheel_radius_ / wheel_separation_;
  
  // 🔍 DEBUG: Show conversion results
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
    "🔄 Converted to robot motion: Linear=%.3f m/s, Angular=%.3f rad/s", 
    linear_vel, angular_vel);
  
  // Create and publish cmd_vel message
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = linear_vel;
  twist_msg.angular.z = angular_vel;
  
  // 🔍 DEBUG: Show what we're publishing
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
    "📡 Publishing to /cmd_vel: x=%.3f, z=%.3f", 
    twist_msg.linear.x, twist_msg.angular.z);
  
  cmd_vel_pub_->publish(twist_msg);

  return hardware_interface::return_type::OK;
}

void RoombaHardwareInterface::sensor_callback(const roomba_interfaces::msg::SensorData::SharedPtr msg)
{
  // Update encoder positions (accumulate distance traveled)
  if (!sensor_data_received_)
  {
    // First reading - initialize
    last_left_encoder_ = msg->left_encoder;
    last_right_encoder_ = msg->right_encoder;
    left_encoder_pos_ = 0.0;
    right_encoder_pos_ = 0.0;
    sensor_data_received_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), 
      "📊 First sensor reading: L=%.2f, R=%.2f cm", 
      msg->left_encoder, msg->right_encoder);
  }
  else
  {
    // Accumulate distance
    double left_delta = msg->left_encoder - last_left_encoder_;
    double right_delta = msg->right_encoder - last_right_encoder_;
    
    left_encoder_pos_ += left_delta;
    right_encoder_pos_ += right_delta;
    
    last_left_encoder_ = msg->left_encoder;
    last_right_encoder_ = msg->right_encoder;
  }
  
  last_sensor_time_ = node_->get_clock()->now();
}

}  // namespace roomba_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roomba_hardware::RoombaHardwareInterface, hardware_interface::SystemInterface)