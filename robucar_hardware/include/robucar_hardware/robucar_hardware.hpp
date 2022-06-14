#ifndef ROBUCAR_HARDWARE_HPP_
#define ROBUCAR_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include "communication/pure_client.hpp"

#include <rclcpp/macros.hpp>

#include <fstream>

namespace romea
{

class RobucarHardware : public HardwareSystemInterface2AS4WD
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(RobucarHardware);

  RobucarHardware();

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;


private:

  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
      const hardware_interface::HardwareInfo & hardware_info) override ;

  bool is_drive_enable() const;

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:

  std::string robot_ip_;
  robucar_communication::PureClient& pure_client_;
  robucar_communication::NotificationEntry* notifier_;
  const uint16_t pure_target_;

  double front_wheel_radius_;
  double rear_wheel_radius_;

  double front_steering_angle_measure_;
  double rear_steering_angle_measure_;
  double front_left_wheel_speed_measure_;
  double front_right_wheel_speed_measure_;
  double rear_left_wheel_speed_measure_;
  double rear_right_wheel_speed_measure_;

  double front_steering_angle_command_;
  double rear_steering_angle_command_;
  double front_left_wheel_speed_command_;
  double front_right_wheel_speed_command_;
  double rear_left_wheel_speed_command_;
  double rear_right_wheel_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

}

#endif
