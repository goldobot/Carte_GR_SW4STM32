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
		static constexpr size_t c_buffer_size = 255;
		void taskFunction() override;
		void printf(const char* format...);

		char read_char();
		const char* read_line();

		bool peek_char(char* c);
		bool prompt_char(const char* prompt, char* c);
		bool prompt_int(const char* prompt, int* c);

		void loop_test_encoders();
		void loop_test_motors();
		void loop_test_odometry();
		void loop_calibrate_odometry();
		void loop_measure_encoders_delta(int32_t* left, int32_t* right);

		char m_buffer[c_buffer_size+1];
		uint8_t m_buffer_index;
	};
}
