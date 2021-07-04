#include "goldobot/platform/hal_gpio.hpp"

extern "C" {
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
}

namespace goldobot {
namespace hal {
namespace platform {
GPIO_TypeDef* g_gpio_ports[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};
GpioDevice g_gpio_devices[32];
}  // namespace platform

using namespace platform;

void gpio_set(int gpio_index, bool value) {
  auto& gpio_device = g_gpio_devices[gpio_index];
  if (!(gpio_device.flags & 0x02)) {
    return;
  }
  auto gpio_port = g_gpio_ports[gpio_device.port];
  uint32_t pin = (uint32_t)(1U << gpio_device.pin);

  if (value) {
    HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(gpio_port, pin, GPIO_PIN_RESET);
  }
}

bool gpio_get(int gpio_index) {
  auto& gpio_device = g_gpio_devices[gpio_index];
  if (!(gpio_device.flags & 0x01)) {
    return false;
  }
  auto gpio_port = g_gpio_ports[gpio_device.port];
  uint32_t pin = (uint32_t)(1U << gpio_device.pin);

  return HAL_GPIO_ReadPin(gpio_port, pin) == GPIO_PIN_SET;
}

namespace platform {

struct AFDef {
  uint8_t signal;
  uint8_t port;
  uint8_t pin;
  uint8_t af;
};

const AFDef g_TIM1_alternate_functions[] = {0,    0, 8,  GPIO_AF6_TIM1,   // PA8
                                            0,    2, 0,  GPIO_AF2_TIM1,   // PC0
                                            1,    0, 7,  GPIO_AF6_TIM1,   // PA7
                                            1,    0, 11, GPIO_AF6_TIM1,   // PA11
                                            1,    1, 13, GPIO_AF6_TIM1,   // PB13
                                            1,    2, 13, GPIO_AF4_TIM1,   // PC13
                                            2,    0, 9,  GPIO_AF6_TIM1,   // PA9
                                            2,    2, 1,  GPIO_AF2_TIM1,   // PC1
                                            3,    0, 12, GPIO_AF6_TIM1,   // PA12
                                            3,    1, 0,  GPIO_AF6_TIM1,   // PB0
                                            3,    1, 14, GPIO_AF6_TIM1,   // PB14
                                            4,    0, 10, GPIO_AF6_TIM1,   // PA10
                                            4,    2, 2,  GPIO_AF2_TIM1,   // PC2
                                            5,    1, 1,  GPIO_AF6_TIM1,   // PB1
                                            5,    1, 15, GPIO_AF4_TIM1,   // PB15
                                            5,    6, 0,  GPIO_AF6_TIM1,   // PF0-OSC_IN
                                            6,    0, 11, GPIO_AF11_TIM1,  // PA11
                                            6,    2, 3,  GPIO_AF2_TIM1,   // PC3
                                            0xff, 0, 0,  0};

const AFDef g_TIM2_alternate_functions[] = {0,    0, 0,  GPIO_AF1_TIM2,   // PA0
                                            0,    0, 5,  GPIO_AF1_TIM2,   // PA5
                                            0,    0, 15, GPIO_AF1_TIM2,   // PA15
                                            2,    0, 1,  GPIO_AF1_TIM2,   // PA1
                                            2,    1, 3,  GPIO_AF1_TIM2,   // PB3
                                            4,    0, 2,  GPIO_AF1_TIM2,   // PA2
                                            4,    0, 9,  GPIO_AF10_TIM2,  // PA9
                                            4,    1, 10, GPIO_AF1_TIM2,   // PB10
                                            6,    0, 3,  GPIO_AF1_TIM2,   // PA3
                                            6,    0, 10, GPIO_AF10_TIM2,  // PA10
                                            6,    1, 11, GPIO_AF1_TIM2,   // PB11
                                            0xff, 0, 0,  0};

const AFDef g_TIM3_alternate_functions[] = {0,    0, 6, GPIO_AF2_TIM3,   // PA6
                                            0,    1, 4, GPIO_AF2_TIM3,   // PB4
                                            0,    2, 6, GPIO_AF2_TIM3,   // PC6
                                            2,    0, 4, GPIO_AF2_TIM3,   // PA4
                                            2,    0, 7, GPIO_AF2_TIM3,   // PA7
                                            2,    1, 5, GPIO_AF2_TIM3,   // PB5
                                            2,    2, 7, GPIO_AF2_TIM3,   // PC7
                                            4,    1, 0, GPIO_AF2_TIM3,   // PB0
                                            4,    2, 8, GPIO_AF2_TIM3,   // PC8
                                            6,    1, 1, GPIO_AF2_TIM3,   // PB1
                                            6,    1, 7, GPIO_AF10_TIM3,  // PB7
                                            6,    2, 9, GPIO_AF2_TIM3,   // PC9
                                            0xff, 0, 0, 0};

const AFDef g_TIM4_alternate_functions[] = {0,    0, 11, GPIO_AF10_TIM4,  // PA11
                                            0,    1, 6,  GPIO_AF2_TIM4,   // PB6
                                            2,    0, 12, GPIO_AF10_TIM4,  // PA12
                                            2,    1, 7,  GPIO_AF2_TIM4,   // PB7
                                            4,    0, 13, GPIO_AF10_TIM4,  // PA13
                                            4,    1, 8,  GPIO_AF2_TIM4,   // PB8
                                            6,    1, 9,  GPIO_AF2_TIM4,   // PB9
                                            0xff, 0, 0,  0};

const AFDef g_TIM6_alternate_functions[] = {0xff, 0, 0, 0};

const AFDef g_TIM7_alternate_functions[] = {0xff, 0, 0, 0};

const AFDef g_TIM8_alternate_functions[] = {0,    0, 15, GPIO_AF2_TIM8,   // PA15
                                            0,    1, 6,  GPIO_AF5_TIM8,   // PB6
                                            0,    2, 6,  GPIO_AF4_TIM8,   // PC6
                                            1,    0, 7,  GPIO_AF4_TIM8,   // PA7
                                            1,    1, 3,  GPIO_AF4_TIM8,   // PB3
                                            1,    2, 10, GPIO_AF4_TIM8,   // PC10
                                            2,    0, 14, GPIO_AF5_TIM8,   // PA14
                                            2,    1, 8,  GPIO_AF10_TIM8,  // PB8
                                            2,    2, 7,  GPIO_AF4_TIM8,   // PC7
                                            3,    1, 0,  GPIO_AF4_TIM8,   // PB0
                                            3,    1, 4,  GPIO_AF4_TIM8,   // PB4
                                            3,    2, 11, GPIO_AF4_TIM8,   // PC11
                                            4,    1, 9,  GPIO_AF10_TIM8,  // PB9
                                            4,    2, 8,  GPIO_AF4_TIM8,   // PC8
                                            5,    1, 1,  GPIO_AF4_TIM8,   // PB1
                                            5,    1, 5,  GPIO_AF3_TIM8,   // PB5
                                            5,    2, 12, GPIO_AF4_TIM8,   // PC12
                                            6,    2, 9,  GPIO_AF4_TIM8,   // PC9
                                            0xff, 0, 0,  0};

const AFDef g_TIM15_alternate_functions[] = {0,    0, 2,  GPIO_AF9_TIM15,  // PA2
                                             0,    1, 14, GPIO_AF1_TIM15,  // PB14
                                             1,    0, 1,  GPIO_AF9_TIM15,  // PA1
                                             1,    1, 15, GPIO_AF2_TIM15,  // PB15
                                             2,    0, 3,  GPIO_AF9_TIM15,  // PA3
                                             2,    1, 15, GPIO_AF1_TIM15,  // PB15
                                             0xff, 0, 0,  0};

const AFDef g_TIM16_alternate_functions[] = {0,    0, 6,  GPIO_AF1_TIM16,  // PA6
                                             0,    0, 12, GPIO_AF1_TIM16,  // PA12
                                             0,    1, 4,  GPIO_AF1_TIM16,  // PB4
                                             0,    1, 8,  GPIO_AF1_TIM16,  // PB8
                                             1,    0, 13, GPIO_AF1_TIM16,  // PA13
                                             1,    1, 6,  GPIO_AF1_TIM16,  // PB6
                                             0xff, 0, 0,  0};

const AFDef g_TIM17_alternate_functions[] = {0,    0, 7, GPIO_AF1_TIM17,   // PA7
                                             0,    1, 5, GPIO_AF10_TIM17,  // PB5
                                             0,    1, 9, GPIO_AF1_TIM17,   // PB9
                                             1,    1, 7, GPIO_AF1_TIM17,   // PB7
                                             0xff, 0, 0, 0};

const AFDef g_TIM20_alternate_functions[] = {0xff, 0, 0, 0};

const AFDef g_CAN_alternate_functions[] = {0,    0, 11, GPIO_AF9_CAN,  // PA11
                                           0,    1, 8,  GPIO_AF9_CAN,  // PB8
                                           1,    0, 12, GPIO_AF9_CAN,  // PA12
                                           1,    1, 9,  GPIO_AF9_CAN,  // PB9
                                           0xff, 0, 0,  0};

const AFDef g_I2C1_alternate_functions[] = {0,    0, 15, GPIO_AF4_I2C1,  // PA15
                                            0,    1, 6,  GPIO_AF4_I2C1,  // PB6
                                            0,    1, 8,  GPIO_AF4_I2C1,  // PB8
                                            1,    0, 14, GPIO_AF4_I2C1,  // PA14
                                            1,    1, 7,  GPIO_AF4_I2C1,  // PB7
                                            1,    1, 9,  GPIO_AF4_I2C1,  // PB9
                                            0xff, 0, 0,  0};

const AFDef g_I2C2_alternate_functions[] = {0,    0, 9,  GPIO_AF4_I2C2,  // PA9
                                            0,    6, 1,  GPIO_AF4_I2C2,  // PF1-OSC_OUT
                                            1,    0, 10, GPIO_AF4_I2C2,  // PA10
                                            1,    6, 0,  GPIO_AF4_I2C2,  // PF0-OSC_IN
                                            0xff, 0, 0,  0};

const AFDef g_I2C3_alternate_functions[] = {0,    0, 8, GPIO_AF3_I2C3,  // PA8
                                            1,    1, 5, GPIO_AF8_I2C3,  // PB5
                                            1,    2, 9, GPIO_AF3_I2C3,  // PC9
                                            0xff, 0, 0, 0};

const AFDef g_SPI1_alternate_functions[] = {2,    0, 6,  GPIO_AF5_SPI1,  // PA6
                                            2,    1, 4,  GPIO_AF5_SPI1,  // PB4
                                            1,    0, 7,  GPIO_AF5_SPI1,  // PA7
                                            1,    1, 5,  GPIO_AF5_SPI1,  // PB5
                                            3,    0, 4,  GPIO_AF5_SPI1,  // PA4
                                            3,    0, 15, GPIO_AF5_SPI1,  // PA15
                                            0,    0, 5,  GPIO_AF5_SPI1,  // PA5
                                            0,    1, 3,  GPIO_AF5_SPI1,  // PB3
                                            0xff, 0, 0,  0};

const AFDef g_SPI2_alternate_functions[] = {2,    0, 10, GPIO_AF5_SPI2,  // PA10
                                            2,    1, 14, GPIO_AF5_SPI2,  // PB14
                                            1,    0, 11, GPIO_AF5_SPI2,  // PA11
                                            1,    1, 15, GPIO_AF5_SPI2,  // PB15
                                            3,    1, 12, GPIO_AF5_SPI2,  // PB12
                                            3,    6, 0,  GPIO_AF5_SPI2,  // PF0-OSC_IN
                                            0,    1, 13, GPIO_AF5_SPI2,  // PB13
                                            0,    6, 1,  GPIO_AF5_SPI2,  // PF1-OSC_OUT
                                            0xff, 0, 0,  0};

const AFDef g_SPI3_alternate_functions[] = {2,    1, 4,  GPIO_AF6_SPI3,  // PB4
                                            2,    2, 11, GPIO_AF6_SPI3,  // PC11
                                            1,    1, 5,  GPIO_AF6_SPI3,  // PB5
                                            1,    2, 12, GPIO_AF6_SPI3,  // PC12
                                            3,    0, 4,  GPIO_AF6_SPI3,  // PA4
                                            3,    0, 15, GPIO_AF6_SPI3,  // PA15
                                            0,    1, 3,  GPIO_AF6_SPI3,  // PB3
                                            0,    2, 10, GPIO_AF6_SPI3,  // PC10
                                            0xff, 0, 0,  0};

const AFDef g_USART1_alternate_functions[] = {0,    0, 10, GPIO_AF7_USART1,  // PA10
                                              0,    1, 7,  GPIO_AF7_USART1,  // PB7
                                              0,    2, 5,  GPIO_AF7_USART1,  // PC5
                                              1,    0, 9,  GPIO_AF7_USART1,  // PA9
                                              1,    1, 6,  GPIO_AF7_USART1,  // PB6
                                              1,    2, 4,  GPIO_AF7_USART1,  // PC4
                                              0xff, 0, 0,  0};

const AFDef g_USART2_alternate_functions[] = {0,    0, 3,  GPIO_AF7_USART2,  // PA3
                                              0,    0, 15, GPIO_AF7_USART2,  // PA15
                                              0,    1, 4,  GPIO_AF7_USART2,  // PB4
                                              1,    0, 2,  GPIO_AF7_USART2,  // PA2
                                              1,    0, 14, GPIO_AF7_USART2,  // PA14
                                              1,    1, 3,  GPIO_AF7_USART2,  // PB3
                                              0xff, 0, 0,  0};

const AFDef g_USART3_alternate_functions[] = {0,    1, 8,  GPIO_AF7_USART3,  // PB8
                                              0,    1, 11, GPIO_AF7_USART3,  // PB11
                                              0,    2, 11, GPIO_AF7_USART3,  // PC11
                                              1,    1, 9,  GPIO_AF7_USART3,  // PB9
                                              1,    1, 10, GPIO_AF7_USART3,  // PB10
                                              1,    2, 10, GPIO_AF7_USART3,  // PC10
                                              0xff, 0, 0,  0};

const AFDef g_UART4_alternate_functions[] = {0,    2, 11, GPIO_AF5_UART4,  // PC11
                                             1,    2, 10, GPIO_AF5_UART4,  // PC10
                                             0xff, 0, 0,  0};

const AFDef g_UART5_alternate_functions[] = {0,    3, 2,  GPIO_AF5_UART5,  // PD2
                                             1,    2, 12, GPIO_AF5_UART5,  // PC12
                                             0xff, 0, 0,  0};

const AFDef* g_periph_alternate_functions_ptrs[] = {
    g_TIM1_alternate_functions,   g_TIM2_alternate_functions,   g_TIM3_alternate_functions,
    g_TIM4_alternate_functions,   g_TIM6_alternate_functions,   g_TIM7_alternate_functions,
    g_TIM8_alternate_functions,   g_TIM15_alternate_functions,  g_TIM16_alternate_functions,
    g_TIM17_alternate_functions,  g_TIM20_alternate_functions,  g_CAN_alternate_functions,
    g_I2C1_alternate_functions,   g_I2C2_alternate_functions,   g_I2C3_alternate_functions,
    g_SPI1_alternate_functions,   g_SPI2_alternate_functions,   g_SPI3_alternate_functions,
    g_USART1_alternate_functions, g_USART2_alternate_functions, g_USART3_alternate_functions,
    g_UART4_alternate_functions,  g_UART5_alternate_functions};

uint32_t hal_gpio_get_pin_af(DeviceId device, int signal, PinID pin);

void hal_gpio_init(const DeviceConfigGpio* config) {
  GpioDevice* gpio_ptr = &g_gpio_devices[config->id];

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Alternate = 0;

  if (config->dir & 0x04) {
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  }

  switch (config->dir & 0x03) {
    case 0:
      GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
      gpio_ptr->flags = 0x01;
      break;
    case 1:
      GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
      GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      gpio_ptr->flags = 0x03;
      break;
    case 2:
      GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
      GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
      gpio_ptr->flags = 0x03;
      break;
    default:
      break;
  }

  hal_gpio_init_pin(config->pin, GPIO_InitStruct);
  gpio_ptr->port = config->pin.port;
  gpio_ptr->pin = config->pin.pin;
}

bool hal_gpio_init_pin(PinID pin, const LL_GPIO_InitTypeDef& init) {
  int port_index = pin.port;

  if (port_index == 0xff) {
    return false;
  }

  LL_GPIO_InitTypeDef init2 = init;
  init2.Pin = (uint32_t)(1U << pin.pin);

  switch (port_index) {
    case 0:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
      LL_GPIO_Init(GPIOA, &init2);
      break;
    case 1:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
      LL_GPIO_Init(GPIOB, &init2);
      break;
    case 2:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
      LL_GPIO_Init(GPIOC, &init2);
      break;
    case 3:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
      LL_GPIO_Init(GPIOD, &init2);
      break;
    case 4:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
      LL_GPIO_Init(GPIOE, &init2);
      break;
    case 5:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
      LL_GPIO_Init(GPIOF, &init2);
      break;
    case 6:
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
      LL_GPIO_Init(GPIOG, &init2);
      break;
    default:
      break;
  }
  return true;
}

bool hal_gpio_init_pin_af(DeviceId device, int signal, PinID pin, const LL_GPIO_InitTypeDef& init) {
  int port_index = pin.port;

  if (port_index == 0xff) {
    return false;
  }

  LL_GPIO_InitTypeDef init2 = init;
  init2.Pin = (uint32_t)(1U << pin.pin);
  init2.Alternate = hal_gpio_get_pin_af(device, signal, pin);
  return hal_gpio_init_pin(pin, init2);
}

uint32_t hal_gpio_get_pin_af(DeviceId device, int signal, PinID pin) {
  const AFDef* ptr = g_periph_alternate_functions_ptrs[(int)device - 2];
  while (ptr->signal != 0xff) {
    if (ptr->signal == signal && ptr->port == pin.port && ptr->pin == pin.pin) {
      return ptr->af;
    }
    ptr++;
  }
  assert(false);
  return 0;
}

void hal_gpio_pin_set(PinID pin, bool value) {
  if (pin.port == 0xff) {
    return;
  }

  auto gpio_port = g_gpio_ports[pin.port];
  uint32_t pin_ = (uint32_t)(1U << pin.pin);

  if (value) {
    HAL_GPIO_WritePin(gpio_port, pin_, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(gpio_port, pin_, GPIO_PIN_RESET);
  }
}

bool hal_gpio_pin_get(PinID pin) {
  if (pin.port == 0xff) {
    return false;
  }

  auto gpio_port = g_gpio_ports[pin.port];
  uint32_t pin_ = (uint32_t)(1U << pin.pin);
  return HAL_GPIO_ReadPin(gpio_port, pin_);
}

}  // namespace platform
}  // namespace hal
};  // namespace goldobot
