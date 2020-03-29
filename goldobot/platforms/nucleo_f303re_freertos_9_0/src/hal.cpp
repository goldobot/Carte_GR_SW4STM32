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

uint8_t g_uart2_rx_buffer[512];

//! \brief tx circular buffer
uint8_t g_uart2_tx_buffer[512];


unsigned g_uart2_rx_read_idx = 0;

//! \brief index of next byte to write in tx buffer
unsigned g_uart2_tx_write_idx=0;

//! \brief index of last byte in circular buffer
unsigned g_uart2_tx_read_idx=0;

//! \brief index of last byte of current dma transfer
unsigned g_uart2_tx_dma_end_idx=0;

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
	{GPIOC, GPIO_PIN_6}, //side selection
	{GPIOC, GPIO_PIN_8}, //autoconfig
	{GPIOA, GPIO_PIN_11}, //ev 1
	{GPIOB, GPIO_PIN_0} //ev 2
};


static SemaphoreHandle_t s_uart_semaphore;

extern "C"
{
	extern UART_HandleTypeDef huart1;
	extern UART_HandleTypeDef huart2;
	extern UART_HandleTypeDef huart3;

	extern DMA_HandleTypeDef hdma_usart2_tx;

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

extern "C" {
	void goldo_hal_foo();
}

void goldo_hal_foo()
{
}

void hal_on_dma_cplt (struct __DMA_HandleTypeDef * hdma)
{
	USART2->CR3 &= ~USART_CR3_DMAT;
	g_uart2_tx_read_idx = g_uart2_tx_dma_end_idx < sizeof(g_uart2_tx_buffer) ? g_uart2_tx_dma_end_idx : 0;
	if(g_uart2_tx_write_idx > g_uart2_tx_read_idx)
	{
		g_uart2_tx_dma_end_idx = g_uart2_tx_write_idx > g_uart2_tx_read_idx ? g_uart2_tx_write_idx : sizeof(g_uart2_tx_buffer);
		HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)(g_uart2_tx_buffer + g_uart2_tx_read_idx), (uint32_t)&USART2->TDR, g_uart2_tx_dma_end_idx - g_uart2_tx_read_idx);
	    USART2->ICR = USART_ICR_TCCF;
	    USART2->CR3 |= USART_CR3_DMAT;
	}
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

	s_uart_semaphore = xSemaphoreCreateBinary();

	// Setup uart2 for DMA circular read
	DMA1_Channel6->CPAR = (uint32_t)&USART2->RDR;
	DMA1_Channel6->CMAR = (uint32_t)g_uart2_rx_buffer;
	DMA1_Channel6->CNDTR = 256;
	DMA1_Channel6->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;


	// Setup uart2 for write
	DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;
	DMA1_Channel7->CMAR = (uint32_t)g_uart2_tx_buffer;
	DMA1_Channel7->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR;

	//USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
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

void Hal::set_servo_pwm(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
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


uint8_t* Hal::uart_lock_read(size_t& buffer_size)
{
	int idx = sizeof(g_uart2_rx_buffer) - DMA1_Channel6->CNDTR;

	if(idx >= g_uart2_rx_read_idx)
	{
		if(buffer_size > idx - g_uart2_rx_read_idx)
		{
			buffer_size = idx - g_uart2_rx_read_idx;
		}
		return g_uart2_rx_buffer + g_uart2_rx_read_idx;
	} else
	{
		if(buffer_size > sizeof(g_uart2_rx_buffer) - idx)
		{
			buffer_size = sizeof(g_uart2_rx_buffer) - idx;
		}
		return g_uart2_rx_buffer + g_uart2_rx_read_idx;
	}
}


void Hal::uart_unlock_read(size_t buffer_size)
{
	g_uart2_rx_read_idx += buffer_size;
	if(g_uart2_rx_read_idx == sizeof(g_uart2_rx_buffer))
	{
		g_uart2_rx_read_idx = 0;
	}
}


uint8_t* Hal::uart_lock_write(size_t& buffer_size)
{
	size_t max_size = g_uart2_tx_write_idx >= g_uart2_tx_read_idx ? sizeof(g_uart2_tx_buffer) - g_uart2_tx_write_idx : g_uart2_tx_read_idx - g_uart2_tx_write_idx -1;

	if(buffer_size > max_size)
	{
		buffer_size = max_size;
	}
	return g_uart2_tx_buffer + g_uart2_tx_write_idx;
}


void Hal::uart_unlock_write(size_t written_size)
{
	g_uart2_tx_write_idx += written_size;
	if(g_uart2_tx_write_idx == sizeof(g_uart2_tx_buffer))
	{
		g_uart2_tx_write_idx = 0;
	}

	// If no DMA transfer is currently occuring, start DMA transfer
	if(hdma_usart2_tx.State == HAL_DMA_STATE_READY && g_uart2_tx_write_idx != g_uart2_tx_read_idx)
	{
		hdma_usart2_tx.XferCpltCallback = &hal_on_dma_cplt;
		g_uart2_tx_dma_end_idx = g_uart2_tx_write_idx > g_uart2_tx_read_idx ? g_uart2_tx_write_idx : sizeof(g_uart2_tx_buffer);
		HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)(g_uart2_tx_buffer + g_uart2_tx_read_idx), (uint32_t)&USART2->TDR, g_uart2_tx_dma_end_idx - g_uart2_tx_read_idx);

	    //USART2->ICR = USART_ICR_TCCF;
	    USART2->CR3 |= USART_CR3_DMAT;
	}

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
	return true;
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
