#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/comm_serializer.hpp"
#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot
{
	enum class CommMessageType : uint16_t
	{
		Sync=0,// "goldobot" synchronization message, used to synchronize stream parser
		Heartbeat=1,// Current OS time in ms as uint32, sent every second
		Reset=2,// Sent once on startup
		PropulsionTelemetry=3, //
		StartOfMatch=4,
		EndOfMatch=5,
		PropulsionTelemetryEx=6,
		CmdEmergencyStop=16, // Order an emergency stop
		CmdSelectSide=17, // Select side. payload is an unsigned byte, 0=green, 1=orange
		OdometryConfig=32,
		PropulsionConfig=33,
		SetMotorsEnable=64,
		DebugExecuteTrajectory=65
	};
	class UARTCommTask : public Task
	{
	public:
		UARTCommTask();
		const char* name() const override;

		bool send_message(uint16_t type, const char* buffer, uint16_t size);

	private:
		static constexpr uint16_t c_buffer_size = 255;
		void taskFunction() override;
		void printf(const char* format...);

		void process_message(uint16_t message_type);
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



		char m_send_buffer[128];
		char m_recv_buffer[128];
		int m_send_begin_index;
		int m_send_end_index;

		char m_buffer[c_buffer_size+1];
		uint8_t m_buffer_index;

		unsigned char m_serialize_buffer[2048];
		unsigned char m_deserialize_buffer[1024];
		CommSerializer m_serializer;
		CommDeserializer m_deserializer;
		SemaphoreHandle_t m_serializer_mutex;
	};
}
