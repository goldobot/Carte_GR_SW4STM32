#pragma once
#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_io_device.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

extern "C"
{
	#include "stm32f3xx_hal_i2c.h"
}


namespace goldobot { namespace platform {

extern I2C_HandleTypeDef g_i2c_handles[3];

void hal_i2c_init(IODevice* device, const IODeviceConfigI2c* config);

} };
