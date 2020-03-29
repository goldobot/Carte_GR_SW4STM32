#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldo_comm/comm.hpp"
#include "goldobot/core/message_queue.hpp"
#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot
{
	class UARTCommTask : public Task
	{
	public:
		UARTCommTask();
		const char* name() const override;
		void init();

	private:
		void taskFunction() override;

		void process_message(uint16_t message_type);
		bool send_message(CommMessageType msg_type, const char* buffer, uint16_t size);

		uint32_t m_last_timestamp;

		unsigned char m_tmp_buffer[1024];
		unsigned char m_out_buffer[1024];

		goldo_comm::Comm m_comm;


		MessageQueue m_out_queue;
	};
}
