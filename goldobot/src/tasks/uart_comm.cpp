#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include <stdio.h>
#include <stdarg.h>

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
		printf("\n");
		char c;
		get_char("Enter command: ", &c);
		switch(c)
		{
		case '1':
			loop_test_odometry();
			break;
		case '2':
			printf("you want to test motors\n");
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

bool UARTCommTask::get_char(const char* prompt, char* c)
{
	printf(prompt);
	*c = read_char();
	printf("%c\n",*c);
	return true;
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

void UARTCommTask::loop_test_odometry()
{
	printf("Test odometry\n");
	printf("1: Read encoders\n");

	while(1)
	{
		uint16_t left;
		uint16_t right;
		Hal::read_encoders(left, right);
		printf("%i,%i\n",left,right);
	}
}


