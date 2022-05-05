#include "robucar_hardware/robucar_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
RobucarHardware::RobucarHardware():
  HardwareSystemInterface2AS4WD()
{

}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Read data from robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Send command to robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::connect()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Init communication with robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::disconnect()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Close communication with robot");
  return hardware_interface::return_type::OK;
}

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::RobucarHardware, hardware_interface::SystemInterface)
