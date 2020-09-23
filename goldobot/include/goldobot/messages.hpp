#pragma once
#include "goldobot/enums.hpp"

namespace goldobot {
namespace messages {

/**
 *
 *
 */

struct MsgMatchStateChange {
  MatchState match_state;
  Side side;
};

struct PropulsionTelemetry {
  int16_t x;  // quarters of mm
  int16_t y;
  int16_t yaw;
  int16_t speed;     // mm per second
  int16_t yaw_rate;  // mradian per second
  int16_t acceleration;
  int16_t angular_acceleration;
  uint16_t left_encoder;
  uint16_t right_encoder;
  int8_t left_pwm;  // percents
  int8_t right_pwm;
  uint8_t state;
  uint8_t error;
};

struct PropulsionTelemetryEx {
  int16_t target_x;  // quarters of mm
  int16_t target_y;
  int16_t target_yaw;
  int16_t target_speed;     // mm per second
  int16_t target_yaw_rate;  // mradian per second
  int16_t longitudinal_error;
  int16_t lateral_error;
  int16_t yaw_error;
  int16_t speed_error;
  int16_t yaw_rate_error;
  int32_t left_acc;
  int32_t right_acc;
};

}  // namespace messages
}  // namespace goldobot
