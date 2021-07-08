#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/core/trapezoidal_speed_profile.hpp"
#include "goldobot/propulsion/controller_config.hpp"

#include <cstdint>

namespace goldobot {
class LowLevelController {
 public:
  void reset();
  void update(const RobotPose& current_pose, const RobotPose& target_pose);
  void setConfig(const PropulsionLowLevelControllerConfig& config);
  void setPidConfig(const PropulsionLowLevelPIDConfig& config);

  // Config
  PropulsionLowLevelControllerConfig m_config;

  // PID controllers
  // Inner loop speed control PIDs
  PIDController m_yaw_rate_pid;
  PIDController m_speed_pid;

  // Outer loop position control PIDs
  PIDController m_translation_pid;
  PIDController m_yaw_pid;

  // Errors
  float m_lateral_error{0};
  float m_longi_error{0};
  float m_yaw_error{0};
  float m_speed_error{0};
  float m_yaw_rate_error{0};

  uint8_t m_yaw_control_level{0};    // 0: no control, 1: rate control, 2: abs control
  uint8_t m_longi_control_level{0};  // 0: no control, 1: rate control, 2: abs control

  // Control outputs
  float m_left_motor_velocity_input{0};
  float m_right_motor_velocity_input{0};

  float m_left_motor_torque_input{0};
  float m_right_motor_torque_input{0};

  float m_motor_velocity_limit{100.0f};

  float m_left_motor_torque_lim{0};
  float m_right_motor_torque_lim{0};
};
}  // namespace goldobot
