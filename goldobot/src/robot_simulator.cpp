#include "goldobot/robot_simulator.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

using namespace goldobot;

RobotSimulator::RobotSimulator()
    : m_x(0), m_y(0), m_yaw(0), m_speed(0), m_yaw_rate(0), m_left_pwm(0), m_right_pwm(0) {}

uint16_t RobotSimulator::encoderLeft() { return m_left_encoder.m_counts; }
uint16_t RobotSimulator::encoderRight() { return m_right_encoder.m_counts; }

void RobotSimulator::doStep() {
  // Simple formula for speed and yaw, with perfect motor controllers.
  float target_speed = 0.5 * (m_left_pwm + m_right_pwm) * m_config.speed_coeff;
  float target_yaw_rate =
      (m_right_pwm - m_left_pwm) * m_config.speed_coeff / m_config.wheels_spacing;

  if(!m_motors_enable)
  {
	  target_speed = 0;
	  target_yaw_rate = 0;
  }
  // Low pass filter
  target_speed = target_speed * 0.5 + 0.5 * m_speed;
  target_yaw_rate = target_yaw_rate * 0.5 + 0.5 * m_yaw_rate;

  // Limit acceleration
  float acceleration_limit = 2;
  if (target_speed > m_speed + 1e-3 * acceleration_limit) {
    m_speed = m_speed + 1e-3 * acceleration_limit;
  } else if (target_speed < m_speed - 1e-3 * acceleration_limit) {
    m_speed = m_speed - 1e-3 * acceleration_limit;
  } else {
    m_speed = target_speed;
  }

  m_yaw_rate = target_yaw_rate;

  // Update robot pose and wheels distances
  float d_yaw = 1e-3 * m_yaw_rate;
  float d_trans = 1e-3 * m_speed;

  double ux = cos(m_yaw + 0.5 * d_yaw);
  double uy = sin(m_yaw + 0.5 * d_yaw);

  m_x += ux * d_trans;
  m_y += uy * d_trans;
  m_yaw += d_yaw;

  // Clamp yaw
  if (m_yaw > M_PI) {
    m_yaw -= 2 * M_PI;
  }
  if (m_yaw < -M_PI) {
    m_yaw += 2 * M_PI;
  }

  // Update encoders
  float left_encoder_delta =
      (d_trans - 0.5 * d_yaw * m_config.encoders_spacing) * m_config.encoders_counts_per_m;
  float right_encoder_delta =
      (d_trans + 0.5 * d_yaw * m_config.encoders_spacing) * m_config.encoders_counts_per_m;

  m_left_encoder.update(left_encoder_delta, m_config.encoders_period);
  m_right_encoder.update(right_encoder_delta, m_config.encoders_period);
}

void RobotSimulator::Encoder::update(float delta, uint16_t period) {
  m_delta += delta;
  int d_counts = static_cast<int>(std::trunc(m_delta));
  m_delta -= d_counts;
  int new_counts = m_counts + d_counts;
  while (new_counts >= period) {
    new_counts -= period;
  }
  while (new_counts < 0) {
    new_counts += period;
  }
  m_counts = static_cast<uint16_t>(new_counts);
}
