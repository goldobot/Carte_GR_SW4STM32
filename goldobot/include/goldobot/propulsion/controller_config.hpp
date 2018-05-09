#pragma once
#include "goldobot/core/pid_controller.hpp"

namespace goldobot
{
	struct ControllerConfig
	{
		PIDConfig speed_pid_config;
		PIDConfig yaw_rate_pid_config;
		PIDConfig translation_pid_config;
		PIDConfig yaw_pid_config;
		float lookahead_distance;
		float lookahead_time;
		float static_pid_limit;
		float moving_pid_limit;
		float repositioning_pwm_limit;
	};
}
