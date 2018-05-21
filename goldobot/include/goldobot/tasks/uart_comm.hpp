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
		CommStats=7,
		DbgPrintf=8,
		CmdEmergencyStop=16, // Order an emergency stop
		CmdSelectSide=17, // Select side. payload is an unsigned byte, 0=green, 1=orange
		OdometryConfig=32,
		PropulsionConfig=33,
		DynamixelDescr=34,
		DynamixelRegisters=35,
		DbgGetOdometryConfig=64,
		DbgSetOdometryConfig=65,
		DbgGetPropulsionConfig=66,
		DbgSetPropulsionConfig=67,
		DbgReset=68,
		DbgSetPropulsionEnable=69,
		DbgSetMotorsEnable=70,
		DbgSetMotorsPwm=71,
		DbgPropulsionTest=72,
		DbgDynamixelsList=73,
		DbgDynamixelSetTorqueEnable=74,
		DbgDynamixelSetGoalPosition=75,
		DbgDynamixelSetTorqueLimit=76,
		DbgDynamixelReadRegisters=77,
		DbgDynamixelSetRegisters=78,
		DbgPropulsionRelativeTrajectory=80
	};
	class UARTCommTask : public Task
	{
	public:
		UARTCommTask();
		const char* name() const override;

		bool send_message(uint16_t type, const char* buffer, uint16_t size);
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

		char m_printf_buffer[c_printf_buffer_size+1];

		unsigned char m_serialize_buffer[2048];
		unsigned char m_deserialize_buffer[1024];
		CommSerializer m_serializer;
		CommDeserializer m_deserializer;
		SemaphoreHandle_t m_serializer_mutex;
	};
}
