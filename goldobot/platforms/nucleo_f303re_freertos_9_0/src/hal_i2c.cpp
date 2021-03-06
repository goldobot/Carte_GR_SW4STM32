#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_private.hpp"

extern "C" {
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_i2c_ex.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"

void goldobot_hal_i2c_ev_irq_handler(int ioc_index);
void goldobot_hal_i2c_er_irq_handler(int ioc_index);
}

namespace goldobot {
namespace hal {
namespace platform {
I2C_HandleTypeDef g_i2c_handles[3];
}
}  // namespace hal
}  // namespace goldobot

using namespace goldobot::hal::platform;

// IRQ handlers
void goldobot_hal_i2c_ev_irq_handler(int ioc_index) {
  HAL_I2C_EV_IRQHandler(&g_i2c_handles[ioc_index]);
}

void goldobot_hal_i2c_er_irq_handler(int ioc_index) {
  HAL_I2C_ER_IRQHandler(&g_i2c_handles[ioc_index]);
}

namespace goldobot {
namespace hal {
namespace platform {

void hal_i2c_init(IODevice* device, const IODeviceConfigI2c* config) {
  int i2c_index = (int)config->device_id - (int)DeviceId::I2c1;

  IRQn_Type ev_irq_n;
  IRQn_Type er_irq_n;

  I2C_HandleTypeDef* hi2c = &g_i2c_handles[i2c_index];
  I2C_TypeDef* i2c_instance;

  switch (config->device_id) {
    case DeviceId::I2c1:
      i2c_instance = I2C1;
      ev_irq_n = I2C1_EV_IRQn;
      er_irq_n = I2C1_ER_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
      break;
    case DeviceId::I2c2:
      i2c_instance = I2C2;
      ev_irq_n = I2C2_EV_IRQn;
      er_irq_n = I2C2_ER_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
      break;
    case DeviceId::I2c3:
      i2c_instance = I2C3;
      ev_irq_n = I2C3_EV_IRQn;
      er_irq_n = I2C3_ER_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
      break;
    default:
      break;
  }

  device->device_index = i2c_index;

  /* Interrupt Init */
  HAL_NVIC_SetPriority(ev_irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(ev_irq_n);

  HAL_NVIC_SetPriority(er_irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(er_irq_n);

  hi2c->Instance = i2c_instance;
  hi2c->Init.Timing = config->timing;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  hal_gpio_init_pin_af(config->device_id, 0, config->scl_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 1, config->sda_pin, GPIO_InitStruct);

  HAL_I2C_Init(hi2c);
  HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);
}

}  // namespace platform

}  // namespace hal
};  // namespace goldobot
