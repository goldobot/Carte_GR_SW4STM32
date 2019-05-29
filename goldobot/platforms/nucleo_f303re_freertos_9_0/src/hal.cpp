#include "goldobot/hal.hpp"
#include "goldobot/robot_simulator.hpp"
#include "stm32f3xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <errno.h>
#include <math.h>

//#define SIMULATE_ROBOT


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
	{GPIOC, GPIO_PIN_9},//match start //tmp: blue button on nucleo. //C9 in robot
	{GPIOC, GPIO_PIN_14}, // adversary detection on C14
	{GPIOC, GPIO_PIN_5}, //dynamixels direction
	{GPIOC, GPIO_PIN_6} //side selection
};


static SemaphoreHandle_t s_uart_semaphore;

extern "C"
{
	extern UART_HandleTypeDef huart1;
	extern UART_HandleTypeDef huart2;
	extern UART_HandleTypeDef huart3;

	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim4;
	extern TIM_HandleTypeDef htim16;

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
		&huart2,
		&huart1,
		&huart3
};

#ifdef SIMULATE_ROBOT
static RobotSimulator s_robot_simulator;
#endif

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

#ifdef SIMULATE_ROBOT
	// Init simulator
	RobotSimulatorConfig simulator_config;
	simulator_config.speed_coeff = 1.7f; // Measured on big robot
	simulator_config.wheels_spacing = 0.2f;
	simulator_config.encoders_spacing = 0.3f;
	simulator_config.encoders_counts_per_m = 1 / 1.5e-05f;
	s_robot_simulator.m_config = simulator_config;
#endif
}


void Hal::read_encoders(uint16_t& left, uint16_t& right)
{
#ifdef SIMULATE_ROBOT
	left = s_robot_simulator.m_left_encoder;
	right = s_robot_simulator.m_right_encoder;
	return;
#endif
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

void Hal::set_servo_pwm(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
}

#if 0 /* FIXME : DEBUG */
extern bool g_goldo_megakill_switch;
void Hal::disable_motors_pwm()
{
	HAL_GPIO_WritePin(MAXON1_DIR_GPIO_Port, MAXON1_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MAXON2_DIR_GPIO_Port, MAXON2_DIR_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
#endif

void Hal::set_motors_pwm(float left, float right)
{
#if 0 /* FIXME : DEBUG */
    if (g_goldo_megakill_switch) {
        disable_motors_pwm();
        return;
    }
#endif

#ifdef SIMULATE_ROBOT
	s_robot_simulator.m_left_pwm = left;
	s_robot_simulator.m_right_pwm = right;
	s_robot_simulator.do_step();
#endif
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

bool Hal::uart_transmit_dma(int uart_index, const char* buffer, uint16_t size)
{
	auto huart_ptr = g_uart_handles[uart_index];
	if(HAL_UART_Transmit_DMA(huart_ptr, (uint8_t*)buffer, size)!= HAL_OK)
	{
		return false;
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
	if(HAL_UART_Receive_IT(huart_ptr, (uint8_t*)buffer, size)!= HAL_OK)
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

uint16_t Hal::uart_bytes_received(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];
	portDISABLE_INTERRUPTS();
	uint16_t bytes_received = huart_ptr->RxXferSize - huart_ptr->RxXferCount;
	portENABLE_INTERRUPTS();
	return bytes_received;
}
uint16_t Hal::uart_receive_abort(int uart_index)
{
	auto huart_ptr = g_uart_handles[uart_index];

	portDISABLE_INTERRUPTS();
	uint16_t bytes_received = huart_ptr->RxXferSize - huart_ptr->RxXferCount;
	HAL_UART_AbortReceive_IT(huart_ptr);
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

bool Hal::user_flash_erase(int start_page, int num_pages)
{
	return true;
}



