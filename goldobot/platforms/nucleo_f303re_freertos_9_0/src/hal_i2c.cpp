#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_uart.hpp"

extern "C"
{
	#include "stm32f3xx_hal_i2c.h"
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"

    void goldobot_hal_i2c_ev_irq_handler(int ioc_index);
    void goldobot_hal_i2c_er_irq_handler(int ioc_index);
}

namespace goldobot { namespace platform {
	I2C_HandleTypeDef g_i2c_handles[3];
} }; // namespace goldobot::platform


void goldobot_hal_i2c_ev_irq_handler(int ioc_index)
{
	HAL_I2C_EV_IRQHandler(&goldobot::platform::g_i2c_handles[ioc_index]);
}

void goldobot_hal_i2c_er_irq_handler(int ioc_index)
{
	HAL_I2C_ER_IRQHandler(&goldobot::platform::g_i2c_handles[ioc_index]);
}


namespace goldobot { namespace platform {




}}; // namespace goldobot::platform
