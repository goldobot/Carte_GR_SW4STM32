#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

namespace goldobot { namespace platform {

enum class DeviceType : uint8_t
{
	None=0,
	Uart=1,
	I2C=2,
	Spi=3,
	Encoder=4,
	Pwm=5
};

struct IODeviceFlag
{
	static constexpr uint16_t RxBlocking = 1;
	static constexpr uint16_t TxBlocking = 2;
	static constexpr uint16_t UseDMA = 4;
};

struct DeviceConfig
{
	uint16_t struct_size;
	DeviceType device_type;
	uint8_t device_id;
};

struct DeviceConfigEncoder
{
	uint8_t tim_index;
	uint8_t ch1_port;
	uint8_t ch1_pin;
	uint8_t ch2_port;
	uint8_t ch2_pin;
	uint16_t period;

};

struct IODeviceConfig : DeviceConfig
{
	uint16_t rx_buffer_size;
	uint16_t tx_buffer_size;
	uint16_t io_flags;
};

struct IODeviceConfigUART : IODeviceConfig
{
	uint8_t uart_index;
	uint8_t reserved;
	uint8_t rx_port;
	uint8_t rx_pin;
	uint8_t tx_port;
	uint8_t tx_pin;
	uint32_t baudrate;
};




} }// namespace goldobot::platform
