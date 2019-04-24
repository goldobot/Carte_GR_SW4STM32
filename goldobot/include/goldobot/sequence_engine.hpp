#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/trajectory_planner.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"


namespace goldobot
{
struct Op
{
	uint8_t opcode;
	uint8_t arg1;
	uint8_t arg2;
	uint8_t arg3;
};
	class SequenceTask : public Task
	{
	public:
		MainTask();
		const char* name() const override;

		void matchSelectNextObjective();


		void execute_sequence(int id);


		void doStep(); /*< Execute instructions until forced to wait



	private:
		void taskFunction() override;

		State m_match_state;
		Side m_side;
		uint32_t m_start_of_match_time;
		TrajectoryPlanner m_trajectory_planner;
		uint16_t m_current_trajectory_index;
		uint16_t m_current_objective;
		SemaphoreHandle_t m_dbg_message_queue_mutex;
		MessageQueue m_dbg_message_queue;
		unsigned char m_dbg_message_queue_buffer[512];

		Vector2D m_waypoints[128];
		uint8_t m_trajectory_points[256];

		bool m_sequence_active;
		bool m_wait_current_cmd;
		uint16_t m_current_sequence_id;
		uint16_t m_current_command_id;
		uint32_t m_delay_finished_ts;

		unsigned char m_buffer[4096];
		Op* m_ops;
		unsigned char* m_vars;
		unsigned char* m_var_types;


		//organisation of buffer:
		// header, 64 bits (uint16_t size, uint16_t crc16, uint8_t num_seq, num_var, reserved)
		// array of 4bits variable types
		// array of 32bits variables, 32bits aligned
		// array of 16bit sequence start offsets
		// array of 32bits ops, 32bits aligned
	};
}
