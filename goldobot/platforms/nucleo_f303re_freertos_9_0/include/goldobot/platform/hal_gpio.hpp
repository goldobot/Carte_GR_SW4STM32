#pragma once
#include "goldobot/platform/hal_private.hpp"

extern "C" {
#include "stm32f3xx_ll_gpio.h"
}

namespace goldobot {
namespace hal {
namespace platform {

struct DeviceConfigGpio : DeviceConfig {
  uint8_t id;
  uint8_t dir;  // mode
  PinID pin;
};

void hal_gpio_init(const DeviceConfigGpio* config);

bool hal_gpio_init_pin(PinID pin, const LL_GPIO_InitTypeDef& init);
bool hal_gpio_init_pin_af(DeviceId device, int signal, PinID pin, const LL_GPIO_InitTypeDef& init);

void hal_gpio_pin_set(PinID pin, bool value);
bool hal_gpio_pin_get(PinID pin);

extern GpioDevice g_gpio_devices[32];

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
