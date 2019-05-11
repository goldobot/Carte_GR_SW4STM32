#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/comm_serializer.hpp"
#include "goldobot/comm_deserializer.hpp"
#include "goldobot/core/message_queue.hpp"
#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot
{
	class UART2CommTask : public Task
	{
	public:
		UART2CommTask();
		const char* name() const override;
		void init();

	private:
		void taskFunction() override;

		void process_message(uint16_t message_type);
		bool send_message(CommMessageType msg_type, const char* buffer, uint16_t size);

		uint32_t m_last_timestamp;
		uint16_t m_bytes_sent;
		uint16_t m_serialize_buffer_high_watermark;

		char m_send_buffer[128];
		char m_recv_buffer[128];
		unsigned char m_tmp_buffer[768];

		unsigned char m_serialize_buffer[1024];
		unsigned char m_deserialize_buffer[1024];
		unsigned char m_out_buffer[1024];
		CommSerializer m_serializer;
		CommDeserializer m_deserializer;
		MessageQueue m_out_queue;
	};
}
