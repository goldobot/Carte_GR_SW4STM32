#pragma once
#include "goldobot/tasks/task.hpp"
#include <cstdint>

namespace goldobot
{
	class UARTCommTask : public Task
	{
	public:
		UARTCommTask();
		const char* name() const override;


	private:
		static constexpr uint16_t c_buffer_size = 255;
		void taskFunction() override;
		void printf(const char* format...);

		char read_char();
		const char* read_line();

		bool peek_char(char* c);
		bool prompt_char(const char* prompt, char* c);
		bool prompt_int(const char* prompt, int* c);
		bool prompt_float(const char* prompt, float* c);

		void loop_test_encoders();
		void loop_test_motors();
		void loop_test_odometry();
		void loop_calibrate_odometry();
		void loop_measure_encoders_delta(int32_t* left, int32_t* right);
		void loop_test_propulsion();
		void loop_calibrate_propulsion_control();

		void print_propulsion_debug();

		char m_buffer[c_buffer_size+1];
		uint8_t m_buffer_index;
	};
}
