#include "goldobot/hal.hpp"
#include "stm32f3xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

struct GPIODescriptor
{
	GPIO_TypeDef* port;
	uint16_t pin;
};

static GPIODescriptor s_gpio_descriptors[] ={
	{GPIOA, GPIO_PIN_5},//green led
	{GPIOC, GPIO_PIN_13}//match start //tmp: blue button on nucleo
};


static SemaphoreHandle_t s_uart_semaphore;

extern "C"
{
	extern UART_HandleTypeDef huart2;

	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim4;

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
	{
		BaseType_t xHigherPriorityTaskWoken;
		xSemaphoreGiveFromISR(s_uart_semaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
	{
		BaseType_t xHigherPriorityTaskWoken;
		xSemaphoreGiveFromISR(s_uart_semaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

UART_HandleTypeDef* g_uart_handles[] ={
		&huart2
};


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

	s_uart_semaphore = xSemaphoreCreateBinary();
}


void Hal::read_encoders(uint16_t& left, uint16_t& right)
{
	left = 8192 - htim4.Instance->CNT;
	right = 8192 - htim1.Instance->CNT;
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

bool Hal::uart_read_char(int uart_index, char* c, bool blocking)
{
	if(!blocking)
	{
		return HAL_UART_Receive(&huart2, (uint8_t*) c, 1, 0) == HAL_OK;
	}
	while(HAL_UART_Receive(&huart2, (uint8_t*) c, 1, 0) != HAL_OK)
	{
		taskYIELD();
	}
	return true;
}

bool Hal::uart_transmit(int uart_index, const char* buffer, uint16_t size, bool blocking)
{
	auto huart_ptr = g_uart_handles[uart_index];
	if(HAL_UART_Transmit_IT(huart_ptr, (uint8_t*)buffer, size)!= HAL_OK)
	{
		return false;
	}
	if(blocking)
	{
		Hal::uart_wait_for_transmit(uart_index);
	}
	return true;
}

bool Hal::uart_transmit_finished(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];
	return huart_ptr->gState != HAL_UART_STATE_BUSY_TX;
}

void Hal::uart_wait_for_transmit(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];
	while (huart_ptr->gState == HAL_UART_STATE_BUSY_TX)
	{
		// Semaphore is unblocked by UART interrupts.
		xSemaphoreTake(s_uart_semaphore, portMAX_DELAY);
	}
}

bool Hal::uart_receive(int uart_index, const char* buffer, uint16_t size, bool blocking)
{
	auto huart_ptr = g_uart_handles[uart_index];
	if(HAL_UART_Receive_IT(&huart2, (uint8_t*)buffer, size)!= HAL_OK)
	{
		return false;
	}
	if(blocking)
	{
		Hal::uart_wait_for_receive(uart_index);
	}
	return true;
}

bool Hal::uart_receive_finished(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];
	return huart_ptr->RxState != HAL_UART_STATE_BUSY_RX;
}

void Hal::uart_wait_for_receive(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];
	while (huart_ptr->RxState == HAL_UART_STATE_BUSY_RX)
	{
		// Semaphore is unblocked by UART interrupts.
		xSemaphoreTake(s_uart_semaphore, portMAX_DELAY);
	}
}

uint16_t Hal::uart_receive_abort(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];

	portDISABLE_INTERRUPTS();
	uint16_t bytes_received = huart2.RxXferSize - huart2.RxXferCount;
	HAL_UART_AbortReceive_IT(&huart2);
	portENABLE_INTERRUPTS();

	// Unblock tasks potentially waiting for recevie to finish
	xSemaphoreGive(s_uart_semaphore);
	return bytes_received;
}

void Hal::set_gpio(int gpio_index, bool value)
{
	auto& desc = s_gpio_descriptors[gpio_index];
	if(value)
	{
		HAL_GPIO_WritePin(desc.port, desc.pin, GPIO_PIN_SET);
	} else
	{
		HAL_GPIO_WritePin(desc.port, desc.pin, GPIO_PIN_RESET);
	}
}

bool Hal::get_gpio(int gpio_index)
{
	auto& desc = s_gpio_descriptors[gpio_index];
	return HAL_GPIO_ReadPin(desc.port, desc.pin) == GPIO_PIN_SET;
}



