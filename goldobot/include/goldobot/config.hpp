#pragma once
#include "goldobot/enums.hpp"

namespace goldobot {

struct ServoConfig
{
	uint8_t id;
	ServoType type;
	uint16_t cw_limit;
	uint16_t ccw_limit;
	uint16_t max_speed;
};

struct ArmConfig
{
	uint8_t num_servos;
	ServoConfig servos[];
};



}
