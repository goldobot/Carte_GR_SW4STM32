#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/trajectory_planner.hpp"
#include "goldobot/sequence_engine.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"


namespace goldobot
{


	class MainTask : public Task
	{
	public:
		MainTask();
		const char* name() const override;

		// Number of seconds before end of match
		int remainingMatchTime();
		void preMatchBegin();
		void preMatchStep();
	private:

		void taskFunction() override;
		void process_message();

		uint32_t m_start_of_match_time;
		MessageQueue m_message_queue;
		unsigned char m_message_queue_buffer[128];

		SequenceEngine m_sequence_engine;
	};
}
