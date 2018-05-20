#pragma once

#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class Arm
	{
	public:
		void set_time(uint32_t time_ms);

	public:
		uint16_t m_servo_ids[5];
		uint16_t m_servo_positions[5];
		uint16_t m_servo_speeds[5];
		uint16_t m_servo_torques[5];
	};
}
