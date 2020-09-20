#include "goldobot/platform/hal_private.hpp"

#include "stm32f3xx_hal.h"
extern "C"
{
	#include "stm32f3xx_hal_tim.h"
    #include "stm32f3xx_ll_usart.h"
    #include "stm32f3xx_ll_gpio.h"
    #include "stm32f3xx_ll_bus.h"
}

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "core_cm4.h"

#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_timer.hpp"
#include "goldobot/platform/hal_uart.hpp"
#include "goldobot/platform/hal_i2c.hpp"
#include "goldobot/platform/hal_spi.hpp"

#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <errno.h>
#include <math.h>
#include <algorithm>
#include <cstring>

// Configuration structures

extern "C"
{
void __assert_func(const char* filename, int line, const char*, const char*)
{
	while(1)
	{
		__asm__("BKPT");
	}
};
}


using namespace goldobot;
using namespace goldobot::platform;




static void init_io_device(IODeviceConfig* config)
{
	IODevice* device = &g_io_devices[config->io_device_id];
	device->io_flags = config->io_flags;

	if(config->rx_buffer_size > 0)
	{
		uint8_t* rx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
		device->rx_queue.init(rx_buffer, config->rx_buffer_size);
	}

	if(config->rx_buffer_size > 0)
	{
		uint8_t* tx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
		device->tx_queue.init(tx_buffer, config->tx_buffer_size);
	}

	device->rx_semaphore = xSemaphoreCreateBinary();
	device->tx_semaphore = xSemaphoreCreateBinary();

	switch(config->device_type )
	{
	case DeviceType::Uart:
		hal_usart_init(device, static_cast<const IODeviceConfigUart*>(config));
		break;
	case DeviceType::I2c:
		hal_i2c_init(device, static_cast<const IODeviceConfigI2c*>(config));
		break;
	case DeviceType::Spi:
		hal_spi_init(device, static_cast<const IODeviceConfigSpi*>(config));
		break;
	default:
		break;
	}

	// Non blocking io (fifo mode)
	if(!(device->io_flags & IODeviceFlags::RxBlocking))
	{
		device->start_rx_fifo();
	}
}


void Hal::configure(void* config)
{
	uint16_t num_devices = *reinterpret_cast<uint16_t*>(config);
	uint16_t* offsets = reinterpret_cast<uint16_t*>(config + 2);
	for(int i=0; i < num_devices; i++)
	{
		DeviceConfig* device_config = reinterpret_cast<DeviceConfig*>(config + offsets[i]);
		switch(device_config->device_type)
		{
		case DeviceType::Gpio:
			hal_gpio_init(static_cast<DeviceConfigGpio*>(device_config));
			break;
		case DeviceType::Timer:
			hal_timer_init(static_cast<DeviceConfigTimer*>(device_config));
			break;
		case DeviceType::Pwm:
			hal_pwm_init(static_cast<DeviceConfigPwm*>(device_config));
			break;
		case DeviceType::Encoder:
			hal_encoder_init(static_cast<DeviceConfigEncoder*>(device_config));
			break;
		case DeviceType::Uart:
			init_io_device(static_cast<IODeviceConfig*>(device_config));
			break;
		case DeviceType::I2c:
			init_io_device(static_cast<IODeviceConfig*>(device_config));
			break;
		case DeviceType::Spi:
			init_io_device(static_cast<IODeviceConfig*>(device_config));
			break;
		default:
			break;
		}


	}
}
void Hal::init()
{
	hal_callback_handler_task_start();

    // init uart
	IODeviceConfigUart uart_config;
	uart_config.device_type = DeviceType::Uart;
	uart_config.io_device_id = 0;
	uart_config.device_id = DeviceId::Usart2;
	uart_config.baudrate = 500000U; //230400U;
	uart_config.rx_pin = PinID{0,3};
	uart_config.tx_pin = PinID{0,2};
	uart_config.rx_buffer_size = 256;
	uart_config.tx_buffer_size = 256;
	uart_config.io_flags = 0;

	init_io_device(&uart_config);

    
	// Start timers for encoders
	//HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1);
}

TickType_t Hal::get_tick_count()
{
	return xTaskGetTickCount();
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

void Hal::gpio_set(int gpio_index, bool value)
{
	auto& gpio_device = g_gpio_devices[gpio_index];
	auto gpio_port = g_gpio_ports[gpio_device.port];
	uint32_t pin = (uint32_t)( 1U << gpio_device.pin);

	if(value)
	{
		HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_SET);
	} else
	{
		HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_RESET);
	}
}


bool Hal::gpio_get(int gpio_index)
{
	auto& gpio_device = g_gpio_devices[gpio_index];
	auto gpio_port = g_gpio_ports[gpio_device.port];
	uint32_t pin = (uint32_t)( 1U << gpio_device.pin);

	return HAL_GPIO_ReadPin(gpio_port, pin) == GPIO_PIN_SET;
}

void Hal::pwm_set(int pwm_id, float value)
{
	if(g_pwm_channels[pwm_id].timer_id == 0xff)
	{
		return;
	}

	auto htim = &g_tim_handles[g_pwm_channels[pwm_id].timer_id];
	auto descr = g_pwm_channels[pwm_id];

	uint32_t channel;
	switch(descr.channel_id)
	{
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	case 3:
		channel = TIM_CHANNEL_3;
		break;
	case 4:
		channel = TIM_CHANNEL_4;
		break;
	default:
		return;
	}


	uint32_t pwm_raw = 0;
	GPIO_PinState dir_pin_state;

	if(value >= 0)
	{
		pwm_raw = static_cast<uint32_t>(value * htim->Init.Period);
		dir_pin_state = descr.sign_pin & 0x80 ? GPIO_PIN_RESET : GPIO_PIN_SET; // if bit set, gpio = 1 for negative
	} else
	{
		pwm_raw = static_cast<uint32_t>(-value * htim->Init.Period);
		dir_pin_state = descr.sign_pin & 0x80 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	}


	if(pwm_raw > htim->Init.Period)
	{
		pwm_raw = htim->Init.Period;
	}
	__HAL_TIM_SET_COMPARE(htim, channel, pwm_raw);

	if(descr.sign_port != 0xff)
	{
		auto port = g_gpio_ports[descr.sign_port];
		uint32_t pin = (uint32_t)( 1U << descr.sign_pin);
		HAL_GPIO_WritePin(port, pin, dir_pin_state);
	}
}

void Hal::send_spi_frame(unsigned char* buff_out, unsigned char* buff_in)
{
  //HAL_SPI_TransmitReceive_IT(&hspi1, buff_out, buff_in,SPI_FRAME_SZ);
 // while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  //{
  //}
}



