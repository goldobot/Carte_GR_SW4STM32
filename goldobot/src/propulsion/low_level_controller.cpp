#include "goldobot/propulsion/low_level_controller.hpp"

#include <cmath>

namespace goldobot {

void LowLevelController::reset() {
  m_translation_pid.reset();
  m_speed_pid.reset();
  m_yaw_pid.reset();
  m_yaw_rate_pid.reset();
  m_longi_error = 0;
  m_yaw_error = 0;
  m_lateral_error = 0;
  m_speed_error = 0;
  m_yaw_rate_error = 0;
}

void LowLevelController::setConfig(const PropulsionLowLevelControllerConfig& config) {
  m_speed_pid.set_config(config.speed_pid_config);
  m_yaw_rate_pid.set_config(config.yaw_rate_pid_config);
  m_translation_pid.set_config(config.longi_pid_config);
  m_yaw_pid.set_config(config.yaw_pid_config);
}

void LowLevelController::update(const RobotPose& current_pose, const RobotPose& target_pose) {
  // Current robot frame direction
  float ux = cosf(current_pose.yaw);
  float uy = sinf(current_pose.yaw);

  // Compute position error in robot frame, and speed, yaw errors
  float diff_x = (current_pose.position.x - target_pose.position.x);
  float diff_y = (current_pose.position.y - target_pose.position.y);

  m_longi_error = diff_x * ux + diff_y * uy;
  m_lateral_error = -diff_x * uy + diff_y * ux;
  m_yaw_error = angleDiff(current_pose.yaw, target_pose.yaw);
  m_speed_error = current_pose.speed - target_pose.speed;

  // Compute translation and speed command
  // Two nested PID controllers are used
  // First is computing speed correction based on position error
  // Second is computing motors pwm based on speed

  float translation_command = 0;
  float speed_command = 0;

  if (m_longi_control_level >= 2) {
    m_translation_pid.set_target(0, 0);
    translation_command = m_translation_pid.update(m_longi_error);
  }

  if (m_longi_control_level >= 1) {
    m_speed_pid.set_target(target_pose.speed);
    speed_command = m_speed_pid.update(target_pose.speed) + translation_command;
  }

  // Compute yaw and yaw_rate command
  // Two nested PID controllers are used
  // First is computing speed correction based on yaw error
  // Second is computing motors pwm difference based on yaw rate

  float yaw_command = 0;
  float yaw_rate_command = 0;

  if (m_yaw_control_level >= 2) {
    m_yaw_pid.set_target(0);
    yaw_command = m_yaw_pid.update(m_yaw_error);
  }

  if (m_yaw_control_level >= 1) {
    m_yaw_rate_pid.set_target(target_pose.yaw_rate);
    yaw_rate_command = m_yaw_rate_pid.update(current_pose.yaw_rate) + yaw_command;
  }

  m_left_motor_pwm = speed_command - yaw_rate_command;
  m_right_motor_pwm = speed_command + yaw_rate_command;
}
}  // namespace goldobot
