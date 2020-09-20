#pragma once
#include "goldobot/platform/hal_private.hpp"

extern "C"
{
	#include "stm32f3xx_ll_gpio.h"
}

namespace goldobot { namespace platform {

bool hal_gpio_init_pin(PinID pin, const LL_GPIO_InitTypeDef& init);
bool hal_gpio_init_pin_af(DeviceId device, int signal, PinID pin, const LL_GPIO_InitTypeDef& init);

void hal_gpio_pin_set(PinID pin, bool value);


}} //namespace goldobot::platform
