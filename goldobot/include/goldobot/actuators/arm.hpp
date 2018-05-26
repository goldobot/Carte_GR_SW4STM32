#pragma once

#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class Arm
	{
	public:
		enum class State
		{
			Inactive=0,
			Stopped=1,
			Moving=2,
		};
	public:
		void reset(uint32_t time_ms);
		void update(uint32_t time_ms);
		void execute_movement(uint8_t target_position);

	public:
		bool m_command_requested;
		uint16_t* m_positions;
		uint16_t m_servo_ids[5];
		uint16_t m_servo_positions[5];
		uint16_t m_servo_speeds[5];
		uint16_t m_servo_torques[5];
	};
}
