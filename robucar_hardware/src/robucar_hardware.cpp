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


// romea
#include <romea_mobile_base_hardware/hardware_info.hpp>

// ros
#include <rclcpp/rclcpp.hpp>

// std
#include <string>

// local
#include "robucar_hardware/robucar_hardware.hpp"
#include "robucar_hardware/communication/framepure.hpp"

namespace
{
const double WHEEL_LINEAR_SPEED_EPSILON = 0.01;
const double WHEEL_STEERING_ANGLE_EPSILON = 0.03;
}

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
RobucarHardware::RobucarHardware()
: HardwareSystemInterface2AS4WD("RobucarHardware"),
  robot_ip_("192.168.1.2"),
  pure_client_(robucar_communication::PureClient::getInstance(robot_ip_, 60000)),
  notifier_(nullptr),
  pure_target_(2)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
RobucarHardware::~RobucarHardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    front_wheel_radius_ = get_parameter<double>(hardware_info, "front_wheel_radius");
    rear_wheel_radius_ = get_parameter<double>(hardware_info, "rear_wheel_radius");
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type RobucarHardware::read()
#else
hardware_interface::return_type RobucarHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Read data from robot");

  if (notifier_) {
    robucar_communication::OutboundNotificationPtr message = notifier_->pop(1000);
    if (message) {
      if (message->dataLength == 0) {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "RobucarHardware"), "Robot answered inconsistent message... ignored.");
      }

      robucar_communication::FramePureDrive frame_pure(message->dataBuffer, message->dataLength);
      frame_pure.decode();

      front_axle_steering_angle_measure_ = -frame_pure.
        getMotorState(robucar_communication::FramePureDrive::FS)->getPosition();
      rear_axle_steering_angle_measure_ = frame_pure.
        getMotorState(robucar_communication::FramePureDrive::RS)->getPosition();
      // + pour Robucar, - pour Aroco; pour l'angle de braquage arrière

      front_left_wheel_linear_speed_measure_ = frame_pure.
        getMotorState(robucar_communication::FramePureDrive::FL)->getSpeed();
      front_right_wheel_linear_speed_measure_ = frame_pure.
        getMotorState(robucar_communication::FramePureDrive::FR)->getSpeed();
      rear_left_wheel_linear_speed_measure_ = frame_pure.
        getMotorState(robucar_communication::FramePureDrive::RL)->getSpeed();
      rear_right_wheel_linear_speed_measure_ = frame_pure.
        getMotorState(robucar_communication::FramePureDrive::RR)->getSpeed();

      set_hardware_state_();

      std::cout << "angles measures " <<
        front_axle_steering_angle_measure_ << " " <<
        rear_axle_steering_angle_measure_ << std::endl;
      std::cout << "speeds measures " <<
        front_left_wheel_linear_speed_measure_ << " " <<
        front_right_wheel_linear_speed_measure_ << " " <<
        rear_left_wheel_linear_speed_measure_ << " " <<
        rear_right_wheel_linear_speed_measure_ << std::endl;

    } else {
      RCLCPP_WARN(rclcpp::get_logger("RobucarHardware"), "robot timeout.");
    }
  }

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type RobucarHardware::write()
#else
hardware_interface::return_type RobucarHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Send command to robot");

  get_hardware_command_();
  std::cout << "angles command " <<
    front_axle_steering_angle_command_ << " " <<
    rear_axle_steering_angle_command_ << std::endl;
  std::cout << "speeds command" <<
    front_left_wheel_linear_speed_command_ << " " <<
    front_right_wheel_linear_speed_command_ << " " <<
    rear_left_wheel_linear_speed_command_ << " " <<
    rear_right_wheel_linear_speed_command_ << std::endl;
  std::cout << " is_drive_enable()" << is_drive_enable() << std::endl;

  robucar_communication::CommandPureDrive drive(is_drive_enable());
  drive.setFrontSteering(front_axle_steering_angle_command_);
  drive.setRearSteering(rear_axle_steering_angle_command_);
  drive.setSpeedFL(front_left_wheel_linear_speed_command_);
  drive.setSpeedFR(front_right_wheel_linear_speed_command_);
  drive.setSpeedRL(rear_left_wheel_linear_speed_command_);
  drive.setSpeedRR(rear_right_wheel_linear_speed_command_);

  uint8_t buffer[512];
  size_t len = drive.toBuffer(buffer);
  if (pure_client_.sendNotification(pure_target_, static_cast<uint8_t *>(buffer), len) <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "RobucarHardware"), "Cannot send command message to robot through the pure client");
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void RobucarHardware::get_hardware_command_()
{
  core::HardwareCommand2AS4WD command = hardware_interface_->get_command();

  front_axle_steering_angle_command_ = command.frontAxleSteeringAngle;
  rear_axle_steering_angle_command_ = command.frontAxleSteeringAngle;

  front_left_wheel_linear_speed_command_ =
    command.frontLeftWheelSpinningSetPoint * front_wheel_radius_;
  front_right_wheel_linear_speed_command_ =
    command.frontRightWheelSpinningSetPoint * front_wheel_radius_;
  rear_left_wheel_linear_speed_command_ =
    command.rearLeftWheelSpinningSetPoint * rear_wheel_radius_;
  rear_right_wheel_linear_speed_command_ =
    command.rearRightWheelSpinningSetPoint * rear_wheel_radius_;
}

//-----------------------------------------------------------------------------
void RobucarHardware::set_hardware_state_()
{
  core::HardwareState2AS4WD state;

  state.frontAxleSteeringAngle = front_axle_steering_angle_measure_;
  state.rearAxleSteeringAngle = rear_axle_steering_angle_measure_;

  state.frontLeftWheelSpinningMotion.velocity =
    front_left_wheel_linear_speed_measure_ / front_wheel_radius_;
  state.frontRightWheelSpinningMotion.velocity =
    front_right_wheel_linear_speed_measure_ / front_wheel_radius_;
  state.rearLeftWheelSpinningMotion.velocity =
    rear_left_wheel_linear_speed_measure_ / rear_wheel_radius_;
  state.rearRightWheelSpinningMotion.velocity =
    rear_right_wheel_linear_speed_measure_ / rear_wheel_radius_;

  hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
bool RobucarHardware::is_drive_enable() const
{
  float speed_measure = 0.25 * (front_left_wheel_linear_speed_measure_ +
    front_right_wheel_linear_speed_measure_ +
    rear_left_wheel_linear_speed_measure_ +
    rear_right_wheel_linear_speed_measure_);

  float speed_command = 0.25 * (front_left_wheel_linear_speed_command_ +
    front_right_wheel_linear_speed_command_ +
    rear_left_wheel_linear_speed_command_ +
    rear_right_wheel_linear_speed_command_);

  return !(std::abs(front_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(front_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(speed_measure) < WHEEL_LINEAR_SPEED_EPSILON &&
         std::abs(speed_command) < WHEEL_LINEAR_SPEED_EPSILON);
}


//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::connect_()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Init communication with robot");

  if (!notifier_) {
    // make a GET  on the service to determine number of drives
    if (!pure_client_.sendRequest(22, robucar_communication::Pure::ACTION_GET, pure_target_)) {
      RCLCPP_WARN(rclcpp::get_logger("RobucarHardware"), "Cannot send data request to robot.");
    }
    robucar_communication::ResponsePtr response =
      pure_client_.findResponse(22, robucar_communication::Pure::ACTION_GET, pure_target_);
    if (!response) {
      RCLCPP_WARN(rclcpp::get_logger("RobucarHardware"), "Robot has not sent answer.");
    }

    // register, no call-back
    notifier_ = pure_client_.requestNotification(pure_target_, 10, 1);
    if (!notifier_) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("RobucarHardware"),
        "Cannot communicate with robot." <<
          "Try to execute this command to check for network problem:" <<
          "ping " << robot_ip_);
      return hardware_interface::return_type::ERROR;
    }

    // Send null command to be sure that no previous command are kept
    robucar_communication::CommandPureDrive command;
    uint8_t send_buf[512];
    pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));
  }


  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type RobucarHardware::disconnect_()
{
  RCLCPP_INFO(rclcpp::get_logger("RobucarHardware"), "Close communication with robot");

  // send null command
  robucar_communication::CommandPureDrive command;
  uint8_t send_buf[512];
  pure_client_.sendNotification(pure_target_, send_buf, command.toBuffer(send_buf));

  // disable notification
  pure_client_.requestNotificationStop(pure_target_);

  return hardware_interface::return_type::OK;
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void RobucarHardware::open_log_file_()
{
  debug_file_.open(
    std::string("adap2e.dat").c_str(),
    std::fstream::in | std::fstream::out | std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void RobucarHardware::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << "# time, ";
    debug_file_ << " FLS, " << " FRS, ";
    debug_file_ << " RLS, " << " RRS, ";
    debug_file_ << " FSA, " << " RSA, ";
    debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
    debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
    debug_file_ << " FSA_cmd, " << " RSA_cmd, " << "\n";
  }
}

//-----------------------------------------------------------------------------
void RobucarHardware::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count() << " ";
    debug_file_ << front_left_wheel_linear_speed_measure_ << " ";
    debug_file_ << front_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_measure_ << " ";
    debug_file_ << rear_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << front_axle_steering_angle_measure_ << " ";
    debug_file_ << rear_axle_steering_angle_measure_ << " ";
    debug_file_ << front_left_wheel_linear_speed_command_ << " ";
    debug_file_ << front_right_wheel_linear_speed_command_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_command_ << " ";
    debug_file_ << rear_right_wheel_linear_speed_command_ << " ";
    debug_file_ << front_axle_steering_angle_command_ << " ";
    debug_file_ << rear_axle_steering_angle_command_ << " /n";
  }
}
#endif

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::RobucarHardware, hardware_interface::SystemInterface)
