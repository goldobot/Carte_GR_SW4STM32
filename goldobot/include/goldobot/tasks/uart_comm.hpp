#pragma once
#include "goldobot/tasks/task.hpp"
#include <cstdint>

// FreeRTOS task
#include "FreeRTOS.h"
#include "task.h"

namespace goldobot
{
	class UARTCommTask : public Task
	{
	public:
		UARTCommTask();
		const char* name() const override;


	private:
		static constexpr size_t c_buffer_size = 256;
		void taskFunction() override;
		void printf(const char* format...);

		char read_char();
		int read_line();

		bool get_char(const char* prompt, char* c);

		void loop_test_odometry();

		char m_buffer[c_buffer_size];
		uint8_t m_buffer_index;
	};
}
