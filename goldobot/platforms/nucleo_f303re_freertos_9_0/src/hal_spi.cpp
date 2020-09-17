#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_uart.hpp"

extern "C"
{
	#include "stm32f3xx_hal_spi.h"
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"

    void goldobot_hal_spi_irq_handler(int index);
}

namespace goldobot { namespace platform {
	SPI_HandleTypeDef g_spi_handles[4];
} }; // namespace goldobot::platform


void goldobot_hal_spi_irq_handler(int index)
{
	HAL_SPI_IRQHandler(&goldobot::platform::g_spi_handles[index]);
}



namespace goldobot { namespace platform {




}}; // namespace goldobot::platform
