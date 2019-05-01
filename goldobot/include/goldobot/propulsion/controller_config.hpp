#pragma once
#include "goldobot/core/pid_controller.hpp"

namespace goldobot
{
	struct PropulsionLowLevelControllerConfig
	{
		PIDConfig speed_pid_config;
		PIDConfig longi_pid_config;
		PIDConfig yaw_rate_pid_config;
		PIDConfig yaw_pid_config;
	};

	struct PropulsionControllerConfig
	{
		PropulsionLowLevelControllerConfig low_level_config_static;
		PropulsionLowLevelControllerConfig low_level_config_cruise;
		PropulsionLowLevelControllerConfig low_level_config_rotate;
		float lookahead_distance;
		float lookahead_time;
		float static_pwm_limit;
		float cruise_pwm_limit;
		float reposition_pwm_limit;
	};
}
