#include "goldobot/platform/hal_private.hpp"
#include "goldobot/robot_simulator.hpp"

#include "stm32f3xx_hal.h"
extern "C"
{
    #include "stm32f3xx_ll_usart.h"
    #include "stm32f3xx_ll_gpio.h"
    #include "stm32f3xx_ll_bus.h"
}

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "core_cm4.h"

#include "goldobot/platform/hal_io_device.hpp"

#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <errno.h>
#include <math.h>
#include <algorithm>
#include <cstring>

// Configuration structures

namespace goldobot { namespace platform {
void hal_usart_init(IODevice* device, const goldobot::IODeviceConfigUART* config);
} };


extern "C"
{
void __assert_func(const char* filename, int line, const char*, const char*)
{
	while(1)
	{

	}
};
}

//#define SIMULATE_ROBOT

#define SPI_FRAME_SZ 6

#define MAXON_EN_Pin GPIO_PIN_15
#define MAXON_EN_GPIO_Port GPIOC

#define MAXON2_DIR_Pin GPIO_PIN_0
#define MAXON2_DIR_GPIO_Port GPIOA

#define MAXON1_DIR_Pin GPIO_PIN_2
#define MAXON1_DIR_GPIO_Port GPIOB

using namespace goldobot;
using namespace goldobot::platform;

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

extern "C"
{
  extern SPI_HandleTypeDef hspi1;
  I2C_HandleTypeDef hi2c1;
}


extern "C"
{
	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim4;
	extern TIM_HandleTypeDef htim16;
}



static void init_device(IODeviceConfig* config)
{
	IODevice* device = &g_io_devices[config->fd];

	uint8_t* rx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
	uint8_t* tx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));

	device->rx_queue.init(rx_buffer, config->rx_buffer_size);
	device->tx_queue.init(tx_buffer, config->tx_buffer_size);

	device->rx_semaphore = xSemaphoreCreateBinary();
	device->tx_semaphore = xSemaphoreCreateBinary();

	if(config->device_type == IODeviceType::Uart)
	{
		hal_usart_init(device, static_cast<const goldobot::IODeviceConfigUART*>(config));
	}

	// Non blocking io (fifo mode)
	if(true)
	{
		device->start_rx_fifo();
	}
}



void Hal::init()
{
    // init uart
	IODeviceConfigUART uart_config;
	uart_config.device_type = IODeviceType::Uart;
	uart_config.fd = 0;
	uart_config.uart_index = 1; // USART2
	uart_config.baudrate = 230400U;
	uart_config.rx_port = 0;
	uart_config.rx_pin = 3;
	uart_config.tx_port = 0;
	uart_config.tx_pin = 2;
	uart_config.rx_buffer_size = 128;
	uart_config.tx_buffer_size = 128;
	uart_config.flags = 0;

	init_device(&uart_config);

    
	// Start timers for encoders
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1);

	// start timers for motors
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

TickType_t Hal::get_tick_count()
{
	return xTaskGetTickCount();
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



uint16_t Hal::uart_read(int fd, uint8_t* buffer, uint16_t buffer_size)
{
	assert(fd >= 0 && fd < sizeof(g_io_devices)/sizeof(IODevice));
	return g_io_devices[fd].read(buffer, buffer_size);
}

uint16_t Hal::uart_write(int fd, const uint8_t* buffer, uint16_t buffer_size)
{
	assert(fd >= 0 && fd < sizeof(g_io_devices)/sizeof(IODevice));
	return g_io_devices[fd].write(buffer, buffer_size);
}

uint16_t Hal::uart_write_space_available(int fd)
{
	auto& device = g_io_devices[fd];
	return device.tx_queue.space_available();
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


void Hal::send_spi_frame(unsigned char* buff_out, unsigned char* buff_in)
{
  HAL_SPI_TransmitReceive_IT(&hspi1, buff_out, buff_in,SPI_FRAME_SZ);
  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
  }
}



