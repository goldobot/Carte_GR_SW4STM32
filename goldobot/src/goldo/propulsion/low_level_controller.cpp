#include "goldo/propulsion/low_level_controller.hpp"
#include <cmath>

namespace goldobot {

  void LowLevelController::reset()
  {
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

  void LowLevelController::setConfig(const PropulsionLowLevelControllerConfig& config)
  {
    m_speed_pid.set_config(config.speed_pid_config);
    m_yaw_rate_pid.set_config(config.yaw_rate_pid_config);
    m_translation_pid.set_config(config.longi_pid_config);
    m_yaw_pid.set_config(config.yaw_pid_config);
  }

  void LowLevelController::update(const RobotPose& current_pose, const RobotPose& target_pose)
  {
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
    m_yaw_rate_error = target_pose.yaw_rate - current_pose.yaw_rate;

    // Compute translation and speed command
    // Two nested PID controllers are used
    // First is computing speed correction based on position error
    // Second is computing motors pwm based on speed

    float longi_command = 0;
    float speed_error = m_speed_error;

    if(m_longi_control_level >= 2)
    {
      // Position PID outer loop
      longi_command = m_translation_pid.step(m_longi_error);
#if 0 /* FIXME : TODO : is this necessary? (predictive command?) */
      speed_error += out;
#endif
    }

    if(m_longi_control_level >= 1)
    {
      // Speed PID inner loop, plus feedforward
      longi_command +=
        m_speed_pid.step(speed_error) +
        m_speed_pid.config().feed_forward * target_pose.speed;
    }

    // Compute yaw and yaw_rate command
    // Two nested PID controllers are used
    // First is computing speed correction based on yaw error
    // Second is computing motors pwm difference based on yaw rate

    float yaw_command = 0;
    float yaw_rate_error = m_yaw_rate_error;

    if (m_yaw_control_level >= 2)
    {
      // Yaw PID outer loop
      yaw_command = m_yaw_pid.step(m_yaw_error);
#if 0 /* FIXME : TODO : is this necessary? (predictive command?) */
      yaw_rate_error += out;
#endif
    }

    if(m_yaw_control_level >=1)
    {
      // Yaw rate PID inner loop, plus feedforward
      yaw_command +=
        m_yaw_rate_pid.step(yaw_rate_error) +
        m_yaw_rate_pid.config().feed_forward * target_pose.yaw_rate;
    }

#if 0 /* FIXME : TODO : homogenize longi & yaw PID coefficients */
    yaw_command *= m_config.wheels_distance *0.5f;
#endif

    m_left_motor_pwm = longi_command - yaw_command;
    m_right_motor_pwm =  longi_command + yaw_command;
  }

} /* namespace goldobot */
