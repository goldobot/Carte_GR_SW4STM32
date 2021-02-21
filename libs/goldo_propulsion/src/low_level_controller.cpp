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
  m_config = config;
}

void LowLevelController::setPidConfig(const PropulsionLowLevelPIDConfig& config) {
	m_speed_pid.setConfig(config.speed_pid_config);
	  m_yaw_rate_pid.setConfig(config.yaw_rate_pid_config);
	  m_translation_pid.setConfig(config.longi_pid_config);
	  m_yaw_pid.setConfig(config.yaw_pid_config);
}

void LowLevelController::update(const RobotPose& current_pose, const RobotPose& target_pose) {
  // Current robot frame direction
  float ux = cosf(current_pose.yaw);
  float uy = sinf(current_pose.yaw);

  // Compute position error in robot frame, and speed, yaw errors
  float diff_x = (target_pose.position.x - current_pose.position.x);
  float diff_y = (target_pose.position.y - current_pose.position.y);

  m_longi_error = diff_x * ux + diff_y * uy;
  m_lateral_error = -diff_x * uy + diff_y * ux;
  m_yaw_error = angleDiff(target_pose.yaw, current_pose.yaw);
  m_speed_error = target_pose.speed - current_pose.speed;

  // Compute translation and speed command
  // Two nested PID controllers are used
  // First is computing speed correction based on position error
  // Second is computing motors pwm based on speed

  float speed_target = target_pose.speed;
  float speed_error = m_speed_error;
  float speed_command = 0;

  if (m_longi_control_level >= 2) {
	// Position PID outer loop
	float out = m_translation_pid.step(m_longi_error);
	speed_target+= out;
    speed_error += out;
  };

  if (m_longi_control_level >= 1) {
	// Speed PID inner loop, plus feedforward
    speed_command += m_speed_pid.step(speed_error) + speed_target;
  }

  // Compute yaw and yaw_rate command
  // Two nested PID controllers are used
  // First is computing speed correction based on yaw error
  // Second is computing motors pwm difference based on yaw rate

  float yaw_rate_target = target_pose.yaw_rate;
  float yaw_rate_error = 0;
  float yaw_rate_command = 0;

  if (m_yaw_control_level >= 2) {
	  float out = m_yaw_pid.step(m_yaw_error);
	  yaw_rate_target += out;
	  yaw_rate_error += out;
  }

  if (m_yaw_control_level >= 1) {
    yaw_rate_command = m_yaw_rate_pid.step(yaw_rate_error) + yaw_rate_target;
  }

  yaw_rate_command = yaw_rate_command * m_config.wheels_distance *0.5f;

  m_left_motor_pwm = (speed_command - yaw_rate_command) * m_config.motors_speed_factor;
  m_right_motor_pwm = (speed_command + yaw_rate_command) * m_config.motors_speed_factor;
}
}  // namespace goldobot
