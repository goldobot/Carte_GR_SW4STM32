#pragma once
#include "goldobot/core/pid_controller.hpp"

namespace goldobot {

struct PropulsionLowLevelPIDConfig
{
	PIDConfig speed_pid_config;
	PIDConfig longi_pid_config;
	PIDConfig yaw_rate_pid_config;
	PIDConfig yaw_pid_config;
};

struct PropulsionLowLevelControllerConfig {
  float wheels_distance; // distance between wheels
  float motors_speed_factor; // conversion factor between speed in m/s at the wheels and motor velocity setpoint or pwm
};

struct PropulsionControllerConfig {
  PropulsionLowLevelControllerConfig low_level_config;
  PropulsionLowLevelPIDConfig pid_configs[8];
  float lookahead_distance;
  float lookahead_time;
  float static_pwm_limit;
  float cruise_pwm_limit;
  float reposition_pwm_limit;
};
}  // namespace goldobot
