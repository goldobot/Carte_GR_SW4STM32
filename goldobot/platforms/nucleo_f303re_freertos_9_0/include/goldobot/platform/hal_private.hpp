#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

#include "stm32f3xx_hal.h"
extern "C"
{
	#include "stm32f3xx_hal_gpio.h"
	#include "stm32f3xx_hal_tim.h"
}

namespace goldobot { namespace platform {

enum class DeviceId : uint8_t
{
	Gpio=1,
	Tim1,
	Tim2,
	Tim3,
	Tim4,
	Tim6,
	Tim7,
	Tim8,
	Tim15,
	Tim16,
	Tim17,
	Tim20,
	Can,
	I2c1,
	I2c2,
	I2c3,
	Spi1,
	Spi2,
	Spi3,
	Usart1,
	Usart2,
	Usart3,
	Uart4,
	Uart5
};

enum class DeviceType : uint8_t
{
	None=0,
	Gpio,
	Timer,
	Pwm,
	Encoder,
	Uart,
	I2c,
	Spi,
	Can
};

struct IODeviceFlag
{
	static constexpr uint16_t RxBlocking = 1;
	static constexpr uint16_t TxBlocking = 2;
	static constexpr uint16_t UseDMA = 4;
};

struct PinID
{
	uint8_t port;
	uint8_t pin;
};

struct DeviceConfig
{
	uint16_t struct_size;
	DeviceType device_type;
	DeviceId device_id;
};

struct DeviceConfigGpio : DeviceConfig
{
	uint8_t id;
	uint8_t dir;
	PinID pin;
};

struct DeviceConfigTimer : DeviceConfig
{
	uint32_t prescaler;
	uint32_t period;
};

struct DeviceConfigPwm : DeviceConfig
{
	uint8_t pwm_id;
	uint8_t channel;
	PinID pin;
	PinID n_pin;
	PinID dir_pin;
};

struct DeviceConfigEncoder : DeviceConfig
{
	uint8_t tim_index;
	uint8_t flags;
	PinID ch1_pin;
	PinID ch2_pin;
	PinID ch3_pin;
	uint16_t period;
};


struct IODeviceConfig : DeviceConfig
{
	uint8_t io_device_id;
	uint8_t reserved;
	uint16_t rx_buffer_size;
	uint16_t tx_buffer_size;
	uint16_t io_flags;
};

struct IODeviceConfigUART : IODeviceConfig
{
	PinID rx_pin;
	PinID tx_pin;
	uint32_t baudrate;
};

struct GpioDevice
{
	uint8_t port;
	uint8_t pin;
};

extern GPIO_TypeDef* g_gpio_ports[];
extern GpioDevice g_gpio_devices[32];

//GPIO helpers
uint32_t hal_gpio_get_pin_af(DeviceId device, int signal, PinID pin);

// put it here for now
void hal_timer_init(DeviceConfigTimer* config);
void hal_gpio_init(const DeviceConfigGpio* config);
void hal_pwm_init(DeviceConfigPwm* config);


} }// namespace goldobot::platform
