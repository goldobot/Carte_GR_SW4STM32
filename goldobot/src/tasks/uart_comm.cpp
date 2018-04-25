#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace goldobot;

// Temporary
#include "stm32f3xx_hal.h"
extern "C"
{
	UART_HandleTypeDef huart2;
}


UARTCommTask::UARTCommTask()
{
}

const char* UARTCommTask::name() const
{
	return "uart_comm";
}

void UARTCommTask::taskFunction()
{
	setvbuf(stdin, NULL, _IONBF, 0);
	while(1)
	{
		printf("Debug mode\n");
		printf("1: Test encoders \n");
		printf("2: Test motors\n");
		printf("3: Test odometry\n");
		printf("4: Calibrate odometry\n");
		printf("4: Test motion controller\n");
		printf("5: Test path planner\n");
		printf("\n");
		char c = 0;

		prompt_char("Enter command: ", &c);
		switch(c)
		{
		case '1':
			loop_test_encoders();
			break;
		case '2':
			loop_test_motors();
			break;
		case '3':
			loop_test_odometry();
			break;
		case '4':
			loop_calibrate_odometry();
			break;
		default:
			break;
		}
	}
}

void UARTCommTask::printf(const char* format, ...)
{
	// Format string
	va_list args;
	va_start(args, format);
	int num_chars = vsnprintf(m_buffer, c_buffer_size, format, args);
	va_end(args);
	if(num_chars > 0)
	{
		if(HAL_UART_Transmit(&huart2, (uint8_t*)m_buffer, num_chars, 1000)!= HAL_OK)
		{
			return;
		}

		// Wait for transfer complete
		while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
		{
			taskYIELD();
		}
	}
}

bool UARTCommTask::prompt_char(const char* prompt, char* c)
{
	printf(prompt);
	const char* line = read_line();
	bool status = false;
	if(strlen(line) == 1)
	{
		*c = line[0];
		status = true;
	}
	printf("\n");
	return status;
}

bool UARTCommTask::prompt_int(const char* prompt, int* c)
{
	printf(prompt);
	printf("\n");
	const char* line = read_line();
	*c = atoi(line);
	return true;
}

bool UARTCommTask::peek_char(char* c)
{
	return HAL_UART_Receive(&huart2, (uint8_t *) c, 1, 0) == HAL_OK;
}

const char* UARTCommTask::read_line()
{
	m_buffer_index = 0;
	while(m_buffer_index < c_buffer_size)
	{
		char c = read_char();
		if(c == '\r')
		{
			break;
		}

		if(c == '\177' || c == '\b')
		{
			if(m_buffer_index > 0)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1000);
				m_buffer_index--;
			}

		}
		else
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1000);
			m_buffer[m_buffer_index] = c;
			m_buffer_index++;
		}
	}
	m_buffer[m_buffer_index] = '\0';
	return m_buffer;
}

char UARTCommTask::read_char()
{
	char c;
	while(HAL_UART_Receive(&huart2, (uint8_t *) &c, 1, 0) != HAL_OK)
	{
		taskYIELD();
	}
	return c;
}

void UARTCommTask::loop_test_encoders()
{
	printf("Test encoders\n");

	while(1)
	{
		uint16_t left;
		uint16_t right;
		Hal::read_encoders(left, right);
		printf("%i,%i        \r",left,right);

		char c = 0;
		if(peek_char(&c))
		{
			return;
		}
		delayTicks(100);
	}
}


void UARTCommTask::loop_test_odometry()
{
	while(1)
		{
			auto pose = Robot::instance().odometry().pose();
			printf("x= %.3fm, y= %.3fm, yaw=%.1fdeg    \r",pose.position.x, pose.position.y, pose.yaw*180/M_PI);
			//printf("%.3f\n", pose.position.x);
			char c = 0;
			if(peek_char(&c))
			{
				return;
			}
			delayTicks(100);
		}
}


void UARTCommTask::loop_calibrate_odometry()
{
	OdometryConfig odometry_config = Robot::instance().odometryConfig();
	while(1)
	{
		printf("Calibrate odometry\n");
		printf("dist_per_count_left: %.6e\n", odometry_config.dist_per_count_left);
		printf("dist_per_count_right: %.6e\n", odometry_config.dist_per_count_right);
		printf("wheel_spacing: %.6e\n", odometry_config.wheel_spacing);
		printf("1: calibrate translation\n");
		printf("2: calibrate rotation\n");
		printf("q: quit\n");
		char choice = 0;
		prompt_char("Enter command: ", &choice);
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

void UARTCommTask::loop_test_motors()
{
	while(1)
	{
		printf("Test motors\n");
		printf("1: enable motors\n");
		printf("2: disable motors\n");
		printf("3: set pwm [-100-100]\n");
		printf("q: quit\n");
		char choice = 0;
		prompt_char("Enter command: ", &choice);
		switch(choice)
		{
		case '1':
			Hal::set_motors_enable(true);
			break;
		case '2':
			Hal::set_motors_enable(false);
			break;
		case '3':
			int left_pwm, right_pwm;
			prompt_int("Left motor pwm: ",&left_pwm);
			prompt_int("Right motor pwm: ",&right_pwm);
			Hal::set_motors_pwm(left_pwm * 0.01, right_pwm * 0.01);
			break;
		case 'q':
			return;
		}
	}
}
