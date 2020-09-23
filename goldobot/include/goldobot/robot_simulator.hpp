#pragma once

namespace goldobot {
struct RobotSimulatorConfig {
  float speed_coeff;     // wheel speed = speed_coeff * pwm
  float wheels_spacing;  // wheels spacing
  float encoders_spacing;
  float encoders_counts_per_m;
};

class RobotSimulator {
 public:
  RobotSimulator();
  void do_step();

  RobotSimulatorConfig m_config;

  double m_x;
  double m_y;
  double m_yaw;
  double m_speed;
  double m_yaw_rate;

  float m_left_pwm;
  float m_right_pwm;

  double m_left_dist;
  double m_right_dist;

  double m_left_encoder_delta;
  double m_right_encoder_delta;

  int m_left_encoder;
  int m_right_encoder;
};
}  // namespace goldobot
