#ifndef ROOMBA_HARDWARE__ROOMBA_HARDWARE_INTERFACE_HPP_
#define ROOMBA_HARDWARE__ROOMBA_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

// For communicating with your Python nodes
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist.hpp"  // Keep for emergency stop
#include "roomba_interfaces/msg/sensor_data.hpp"
#include "roomba_interfaces/msg/wheel_velocities.hpp"  // NEW: Direct wheel control

namespace roomba_hardware
{
class RoombaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoombaHardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Robot parameters
  double wheel_radius_;
  double wheel_separation_;
  
  // Joint state variables
  double left_wheel_pos_;
  double left_wheel_vel_;
  double right_wheel_pos_;
  double right_wheel_vel_;
  
  // Joint command variables
  double left_wheel_vel_cmd_;
  double right_wheel_vel_cmd_;
  
  // ROS2 communication
  rclcpp::Node::SharedPtr node_;
  
  // UPDATED: Direct wheel velocity publisher (bypasses cmd_vel conversion)
  rclcpp::Publisher<roomba_interfaces::msg::WheelVelocities>::SharedPtr wheel_vel_pub_;
  
  // Keep cmd_vel publisher for emergency stop only
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_stop_pub_;
  
  rclcpp::Subscription<roomba_interfaces::msg::SensorData>::SharedPtr sensor_sub_;
  
  // Sensor data
  double left_encoder_pos_;
  double right_encoder_pos_;
  double last_left_encoder_;
  double last_right_encoder_;
  rclcpp::Time last_sensor_time_;
  bool sensor_data_received_;
  
  void sensor_callback(const roomba_interfaces::msg::SensorData::SharedPtr msg);
};

}  // namespace roomba_hardware

#endif  // ROOMBA_HARDWARE__ROOMBA_HARDWARE_INTERFACE_HPP_