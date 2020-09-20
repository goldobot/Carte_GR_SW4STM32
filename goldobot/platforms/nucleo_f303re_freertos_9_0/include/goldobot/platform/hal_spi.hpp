#pragma once
#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "stm32f3xx_hal.h"

extern "C"
{
	#include "stm32f3xx_hal_spi.h"
}

namespace goldobot { namespace platform {

extern SPI_HandleTypeDef g_spi_handles[3];

void hal_spi_init(IODevice* device, const IODeviceConfigSpi* config);
}} // namespace goldobot::platform
