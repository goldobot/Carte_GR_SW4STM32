#include "goldobot/hal.hpp"
#include "stm32f3xx_hal.h"

#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <errno.h>
#include <math.h>

#define MAXON_EN_Pin GPIO_PIN_15
#define MAXON_EN_GPIO_Port GPIOC

#define MAXON2_DIR_Pin GPIO_PIN_0
#define MAXON2_DIR_GPIO_Port GPIOA

#define MAXON1_DIR_Pin GPIO_PIN_2
#define MAXON1_DIR_GPIO_Port GPIOB
using namespace goldobot;

extern "C"
{
	extern UART_HandleTypeDef huart2;

	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim4;
}

void Hal::init()
{
	// Start timers for encoders
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1);

	// start timers for motors
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Hal::uart_transmit()
{
	//HAL_UART_Transmit(&huart2, (uint8_t*)"foo", 3, 0xFFFF);
}


void Hal::read_encoders(uint16_t& left, uint16_t& right)
{
	left = 8192 - __HAL_TIM_GetCounter(&htim1);
	right = 8192 - __HAL_TIM_GetCounter(&htim4);
}

void Hal::set_motors_enable(bool enabled)
{
	if(enabled)
	{
		HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_SET);
	} else
	{
		HAL_GPIO_WritePin(MAXON_EN_GPIO_Port, MAXON_EN_Pin, GPIO_PIN_RESET);
	}
}

void Hal::set_motors_pwm(float left, float right)
{
	int left_pwm = 0;
	int right_pwm = 0;
	if(left > 0)
	{
		left_pwm = static_cast<int>(left*10000);
		HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_RESET);
	} else
	{
		left_pwm = static_cast<int>(-left*10000);
		HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_SET);
	}

	if(right > 0)
	{
		right_pwm = static_cast<int>(right*10000);
		HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_SET);
	} else
	{
		right_pwm = static_cast<int>(-right*10000);
		HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_RESET);
	}
	if(left_pwm > 10000)
	{
		left_pwm = 10000;
	}
	if(right_pwm > 10000)
	{
		right_pwm = 10000;
	}

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);
}

extern "C"
{
	int _read(int file, char *data, int len)
	{
		if (file != STDIN_FILENO)
	   {
		  errno = EBADF;
		  return -1;
	   }
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);

		// return # of bytes written - as best we can tell
	    return (status == HAL_OK ? len : 0);
	}

	int _write(int file, char *data, int len)
	{
	   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	   {
		  errno = EBADF;
		  return -1;
	   }

	   // arbitrary timeout 1000
	   HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);

	   // return # of bytes written - as best we can tell
	   return (status == HAL_OK ? len : 0);
	}
}
