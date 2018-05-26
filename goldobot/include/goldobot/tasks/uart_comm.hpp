#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/comm_serializer.hpp"
#include "goldobot/message_types.hpp"
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

		bool send_message(CommMessageType msg_type, const char* buffer, uint16_t size);
		void debug_printf(const char* format...);
	private:
		static constexpr uint16_t c_printf_buffer_size = 255;
		void taskFunction() override;


		void process_message(uint16_t message_type);


		uint32_t m_last_timestamp;
		uint16_t m_bytes_sent;
		uint16_t m_serialize_buffer_high_watermark;

		char m_send_buffer[128];
		char m_recv_buffer[128];
		unsigned char m_tmp_buffer[512];

		char m_printf_buffer[c_printf_buffer_size+1];

		unsigned char m_serialize_buffer[2048];
		unsigned char m_deserialize_buffer[1024];
		CommSerializer m_serializer;
		CommDeserializer m_deserializer;
		SemaphoreHandle_t m_serializer_mutex;
	};
}
