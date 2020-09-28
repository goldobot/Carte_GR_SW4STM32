#pragma once
#include <cstdint>

namespace goldobot {
struct RobotSimulatorConfig {
  float speed_coeff;     // wheel speed = speed_coeff * pwm
  float wheels_spacing;  // wheels spacing
  float encoders_spacing;
  float encoders_counts_per_m;
  uint16_t encoders_period;
};

class RobotSimulator {
 public:
  RobotSimulator();
  void doStep();

  uint16_t encoderLeft();
  uint16_t encoderRight();

  RobotSimulatorConfig m_config;

  class Encoder {
   public:
    void update(float delta, uint16_t period);
    float m_delta{0};
    uint16_t m_counts{0};
  };

  double m_x;
  double m_y;
  double m_yaw;
  double m_speed;
  double m_yaw_rate;

  float m_left_pwm;
  float m_right_pwm;

  Encoder m_left_encoder;
  Encoder m_right_encoder;
};
}  // namespace goldobot
