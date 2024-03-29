// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROBUCAR_HARDWARE__ROBUCAR_HARDWARE_HPP_
#define ROBUCAR_HARDWARE__ROBUCAR_HARDWARE_HPP_


// std
#include <fstream>
#include <string>

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

// ros
#include "rclcpp/macros.hpp"

// local
#include "communication/pure_client.hpp"


namespace romea
{
namespace ros2
{

class RobucarHardware : public HardwareSystemInterface2AS4WD
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobucarHardware);

  RobucarHardware();

  virtual ~RobucarHardware();

  #if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;

  bool is_drive_enable() const;

  void get_hardware_command_();

  void set_hardware_state_();

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:
  std::string robot_ip_;
  robucar_communication::PureClient & pure_client_;
  robucar_communication::NotificationEntry * notifier_;
  const uint16_t pure_target_;

  double front_wheel_radius_;
  double rear_wheel_radius_;

  double front_axle_steering_angle_measure_;
  double rear_axle_steering_angle_measure_;
  double front_left_wheel_linear_speed_measure_;
  double front_right_wheel_linear_speed_measure_;
  double rear_left_wheel_linear_speed_measure_;
  double rear_right_wheel_linear_speed_measure_;

  double front_axle_steering_angle_command_;
  double rear_axle_steering_angle_command_;
  double front_left_wheel_linear_speed_command_;
  double front_right_wheel_linear_speed_command_;
  double rear_left_wheel_linear_speed_command_;
  double rear_right_wheel_linear_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

}  // namespace ros2
}  // namespace romea

#endif  // ROBUCAR_HARDWARE__ROBUCAR_HARDWARE_HPP_
