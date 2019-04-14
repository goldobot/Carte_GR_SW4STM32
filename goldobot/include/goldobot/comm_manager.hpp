#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/message_types.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot
{

	class CommManager
	{
	public:
		CommManager();
		void process_messages();

	private:
		bool push_message(uint16_t message_type, const unsigned char* buffer, size_t size);
		void pop_message(unsigned char* buffer, size_t size);

		void process_message(CommMessageType message_type, uint16_t message_size);

		void on_msg_dbg_execute_trajectory();
		void on_msg_dbg_arms_set_pose();
		void on_msg_dbg_arms_set_command();
		void on_msg_dbg_arms_set_sequences(uint16_t message_size);
		void on_msg_dbg_arms_go_to_position();
		void on_msg_dbg_arms_execute_sequence();

		void on_msg_dbg_robot_set_command();
		void on_msg_dbg_robot_set_point();
		void on_msg_dbg_robot_set_sequence();
		void on_msg_dbg_robot_execute_sequence();
		void on_msg_dbg_robot_set_trajectory_point();

		SemaphoreHandle_t m_dbg_message_queue_mutex;
		MessageQueue m_dbg_message_queue;
		unsigned char m_dbg_message_queue_buffer[512];
	};
}
