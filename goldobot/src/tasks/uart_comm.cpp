#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include "stm32f3xx.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;


UARTCommTask::UARTCommTask() :
    m_serializer(m_serialize_buffer, sizeof(m_serialize_buffer)),
	m_deserializer(m_deserialize_buffer, sizeof(m_deserialize_buffer))
{
	m_serializer_mutex = xSemaphoreCreateMutex();
}

const char* UARTCommTask::name() const
{
	return "uart_comm";
}

void UARTCommTask::taskFunction()
{
	set_priority(5);
	m_last_timestamp = xTaskGetTickCount();
	m_bytes_sent = 0;
	m_serialize_buffer_high_watermark = 0;

	Hal::uart_receive(0, m_recv_buffer, sizeof(m_recv_buffer), false);
	while(1)
	{
		// If current transmission is finished, send next chunk of data from ring buffer
		if(Hal::uart_transmit_finished(0))
		{
			if(xSemaphoreTake(m_serializer_mutex, 1) == pdTRUE)
			{
				auto watermark =  m_serializer.size();
				if(watermark > m_serialize_buffer_high_watermark)
				{
					m_serialize_buffer_high_watermark = watermark;
				}
				size_t dtlen = m_serializer.pop_data((unsigned char*)m_send_buffer, sizeof(m_send_buffer));
				if(dtlen)
				{
					Hal::uart_transmit(0, m_send_buffer, dtlen, false);
				}
				m_bytes_sent += dtlen;
				xSemaphoreGive(m_serializer_mutex);
			}
		}
		// Parse received data
		if(Hal::uart_receive_finished(0))
		{
			m_deserializer.push_data((unsigned char*)m_recv_buffer, sizeof(m_recv_buffer));
		} else
		{
			// Abort reception if previous was not finished
			uint16_t bytes_received = Hal::uart_receive_abort(0);
			if(bytes_received)
			{
				m_deserializer.push_data((unsigned char*)m_recv_buffer, bytes_received);
			}
		}
		// Launch new receive command
		Hal::uart_receive(0, m_recv_buffer, sizeof(m_recv_buffer), false);

		// Process received mesage if needed
		if(m_deserializer.message_ready())
		{
			uint16_t message_type = m_deserializer.message_type();
			if(message_type != 0)
			{
				process_message(message_type);
			} else
			{
				m_deserializer.pop_message(nullptr,0);
			}
		}

		uint32_t timestamp = xTaskGetTickCount();
		if(timestamp - m_last_timestamp >= 1000)
		{

			uint16_t msg[2];
			msg[0] = (m_bytes_sent * 1000) / (timestamp - m_last_timestamp);
			msg[1] =  m_serialize_buffer_high_watermark;
			send_message((uint16_t)CommMessageType::CommStats, (char*)msg, 4);
			m_last_timestamp = timestamp;
			m_bytes_sent = 0;
			m_serialize_buffer_high_watermark = 0;
		}
		// Wait for next tick
		delay(1);
	}
}

void UARTCommTask::debug_printf(const char* format, ...)
{
	// Format string
	va_list args;
	va_start(args, format);
	int num_chars = vsnprintf(m_printf_buffer, c_printf_buffer_size, format, args);
	va_end(args);
	if(num_chars > 0)
	{
		send_message((uint16_t)CommMessageType::DbgPrintf, m_printf_buffer, num_chars);
	}
}

bool UARTCommTask::send_message(uint16_t type, const char* buffer, uint16_t size)
{
	if(xSemaphoreTake(m_serializer_mutex, 1) == pdTRUE)
	{
		m_serializer.push_message(type, (const unsigned char*)(buffer), size);
		xSemaphoreGive(m_serializer_mutex);
		return true;
	}
	return false;
}

void UARTCommTask::process_message(uint16_t message_type)
{
	goldobot::PropulsionController* propulsion = &(Robot::instance().propulsion());

	switch((CommMessageType)m_deserializer.message_type())
	{
	case CommMessageType::DbgGetOdometryConfig:
		{
			auto config = Robot::instance().odometry().config();
			send_message((uint16_t)CommMessageType::OdometryConfig, (char*)&config, sizeof(config));
			m_deserializer.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgGetPropulsionConfig:
		{
			auto config = Robot::instance().propulsion().config();
			send_message((uint16_t)CommMessageType::PropulsionConfig, (char*)&config, sizeof(config));
			m_deserializer.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetPropulsionConfig:
		{
			PropulsionControllerConfig config;
			m_deserializer.pop_message((unsigned char*)&config, sizeof(config));
			propulsion->set_config(config);
		}
		break;
	case CommMessageType::DbgReset:
		// Reset the micro
		NVIC_SystemReset();
		break;
	case CommMessageType::CmdEmergencyStop:
		propulsion->emergency_stop();
		m_deserializer.pop_message(nullptr, 0);
		break;
	case CommMessageType::DbgSetPropulsionEnable:
		{
			uint8_t enabled;
			m_deserializer.pop_message((unsigned char*)enabled, 1);
			if(enabled)
			{
				propulsion->enable();
			} else
			{
				propulsion->disable();
			}
		}
		break;
	case CommMessageType::DbgSetMotorsEnable:
		{
			uint8_t enabled;
			m_deserializer.pop_message((unsigned char*)enabled, 1);
			Hal::set_motors_enable(enabled);
		}
		break;
	case CommMessageType::DbgSetMotorsPwm:
		{
			float pwm[2];
			m_deserializer.pop_message((unsigned char*)pwm, 8);
			Hal::set_motors_pwm(pwm[0], pwm[1]);
		}
		break;
	case CommMessageType::DbgPropulsionTest:
		{
			uint8_t id;
			m_deserializer.pop_message((unsigned char*)id, 1);
			switch(id)
			{
			case 0:
				propulsion->executeTest(PropulsionController::TestPattern::SpeedSteps);
				break;
			}
		}
		break;
	case CommMessageType::DbgDynamixelsList:
		{
			m_deserializer.pop_message(nullptr, 0);
			uint8_t buff[4] = {25,1};
			for(unsigned id = 0; id < 0xFE; id++)
			{
				if(Robot::instance().arms().dynamixels_read_data(id,0 , buff, 4))
				{
					send_message((uint16_t)CommMessageType::DynamixelDescr, (char*)buff, 4);
				}
			}
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueEnable:
		{
			unsigned char buff[2];
			m_deserializer.pop_message(buff, 2);
			// Torque enable
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x18, buff+1, 1);
		}
		break;
	case CommMessageType::DbgDynamixelSetGoalPosition:
		{
			unsigned char buff[3];
			m_deserializer.pop_message(buff, 3);
			// Goal position
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x1E, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueLimit:
		{
			unsigned char buff[3];
			m_deserializer.pop_message(buff, 3);
			// Goal position
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x22, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelReadRegisters:
			{
				unsigned char buff[3];
				unsigned char data_read[64];

				m_deserializer.pop_message(buff, 3);
				memcpy(data_read, buff, 2);
				if(Robot::instance().arms().dynamixels_read_data(buff[0], buff[1], data_read+2, buff[2]))
				{
					send_message((uint16_t)CommMessageType::DynamixelRegisters, (char*)data_read, buff[2]+2);
				}
			}
			break;
	case CommMessageType::DbgDynamixelSetRegisters:
		{
			unsigned char buff[128];
			uint16_t size = m_deserializer.message_size();
			m_deserializer.pop_message(buff, 128);
			//id, addr, data
			if(Robot::instance().arms().dynamixels_write_data(buff[0], buff[1], buff+2, size-2));

		}
		break;

	default:
		m_deserializer.pop_message(nullptr, 0);
		break;
	}

	int foo=1;
}
