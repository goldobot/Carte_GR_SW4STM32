#pragma once

extern "C"
{
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"
}

namespace goldobot { namespace platform {

void hal_gpio_init_pin(int port_index, LL_GPIO_InitTypeDef* init);

} } //namespace goldobot::platform
