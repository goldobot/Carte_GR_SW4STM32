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

/*
void UARTCommTask::loop_calibrate_odometry()
{
	OdometryConfig odometry_config = Robot::instance().odometryConfig();
	while(1)
	{
		printf("[Calibrate odometry]\r\n");
		printf("dist_per_count_left: %.6e\r\n", odometry_config.dist_per_count_left);
		printf("dist_per_count_right: %.6e\r\n", odometry_config.dist_per_count_right);
		printf("wheel_spacing: %.6e\r\n", odometry_config.wheel_spacing);
		printf("1: calibrate translation\r\n");
		printf("2: calibrate rotation\r\n");
		printf("q: quit\r\n");
		char choice = 0;
		prompt_char(">: ", &choice);
		switch(choice)
		{
		case '1':
		{
			printf("Advance the robot 50cm and press a key\n");
			int32_t left_total;
			int32_t right_total;
			loop_measure_encoders_delta(&left_total, &right_total);
			printf("counts_left: %i, counts_right: %i\n",left_total,right_total);
			odometry_config.dist_per_count_left = 0.5f / left_total;
			odometry_config.dist_per_count_right = 0.5f / right_total;

		}
			break;
		case '2':
		{
			printf("Rotate the robot one time to the left and press a key\n");
			int32_t left_total;
			int32_t right_total;
			loop_measure_encoders_delta(&left_total, &right_total);
			printf("counts_left: %i, counts_right: %i\n",left_total,right_total);
			float dist = fabs(right_total * odometry_config.dist_per_count_right - left_total * odometry_config.dist_per_count_left);
			odometry_config.wheel_spacing = dist / (M_PI * 2);
		}
		break;
		case 'q':
			return;
		}
	}
}

void UARTCommTask::loop_measure_encoders_delta(int32_t* left_o, int32_t* right_o)
{
	*left_o = 0;
	*right_o = 0;
	uint16_t old_left;
	uint16_t old_right;
	char c = 0;
	Hal::read_encoders(old_left, old_right);
	while(!peek_char(&c) )
	{
		uint16_t left;
		uint16_t right;

		Hal::read_encoders(left, right);

		int diff_left = left - old_left;
		int diff_right = right - old_right;

		old_left = left;
		old_right = right;

		if(diff_left > 4096)
		{
			diff_left -= 8192;
		}
		if(diff_left < -4096)
		{
			diff_left += 8192;
		}
		if(diff_right > 4096)
		{
			diff_right -= 8192;
		}
		if(diff_right < -4096)
		{
			diff_right += 8192;
		}
		*left_o += diff_left;
		*right_o += diff_right;
	}
}
*/

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

	default:
		m_deserializer.pop_message(nullptr, 0);
		break;
	}

	int foo=1;
}
