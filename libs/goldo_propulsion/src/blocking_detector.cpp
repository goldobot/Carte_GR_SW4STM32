#include "goldobot/propulsion/blocking_detector.hpp"
#include "goldobot/propulsion/controller.hpp"

#include <cstdint>

namespace goldobot {
namespace propulsion {

void BlockingDetector::setVelEstimates(float left, float right) {
  m_vel_estimates[0] = left;
  m_vel_estimates[1] = right;
}
void BlockingDetector::setTorqueEstimates(float left, float right) {
  m_torque_estimates[0] = left;
  m_torque_estimates[1] = right;
}

void BlockingDetector::update(const PropulsionController& controller) {
  // compare measured wheel motor speeds to what is predicted by odometry
  float inv_speed_factor = 1 / controller.config().low_level_config.motors_speed_factor;
  float wheels_distance = controller.config().low_level_config.wheels_distance;

  float speed_estimate = (m_vel_estimates[0] + m_vel_estimates[1]) * 0.5f * inv_speed_factor;
  float yaw_rate_estimate_estimate = (m_vel_estimates[1] - m_vel_estimates[0]) * inv_speed_factor;
  float force_estimate = (m_torque_estimates[0] + m_torque_estimates[1]) * 0.5f;

  m_speed_estimate = m_speed_estimate * 0.98f + 0.02f * speed_estimate;
  m_force_estimate = m_force_estimate * 0.98f + 0.02f * force_estimate;
}

/*
float m_vel_estimates[2];
float m_torque_estimates[2];

float wheels_distance;
float motors_speed_factor;
*/

}  // namespace propulsion
}  // namespace goldobot
