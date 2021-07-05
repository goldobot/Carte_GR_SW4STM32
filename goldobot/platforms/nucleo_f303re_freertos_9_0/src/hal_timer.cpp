#include "goldobot/platform/hal_timer.hpp"

#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_private.hpp"

#include "stm32f3xx_hal.h"
extern "C" {
#include "stm32f3xx_hal_tim.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
}

extern "C" {
void goldobot_hal_tim6_irq_handler();
}

bool g_hal_initialized = false;

namespace goldobot {
namespace hal {
namespace platform {

TIM_HandleTypeDef g_tim_handles[11];

PwmChannel g_pwm_channels[8];
DeviceEncoder g_encoders[4];

// TIM1
// TIM2
// TIM3
// TIM4
// TIM6 used as HAL clock source at 1MHz
// TIM7
// TIM8
// TIM15
// TIM16
// TIM17
// TIM20

static TIM_TypeDef* start_timer_clock(DeviceId device_id) {
  switch (device_id) {
    case DeviceId::Tim1:
      __HAL_RCC_TIM1_CLK_ENABLE();
      return TIM1;
      break;
    case DeviceId::Tim2:
      __HAL_RCC_TIM2_CLK_ENABLE();
      return TIM2;
      break;
    case DeviceId::Tim3:
      __HAL_RCC_TIM3_CLK_ENABLE();
      return TIM3;
      break;
    case DeviceId::Tim4:
      __HAL_RCC_TIM4_CLK_ENABLE();
      return TIM4;
      break;
    case DeviceId::Tim7:
      __HAL_RCC_TIM7_CLK_ENABLE();
      return TIM7;
      break;
    case DeviceId::Tim8:
      __HAL_RCC_TIM8_CLK_ENABLE();
      return TIM8;
      break;
    case DeviceId::Tim15:
      __HAL_RCC_TIM15_CLK_ENABLE();
      return TIM15;
      break;
    case DeviceId::Tim16:
      __HAL_RCC_TIM16_CLK_ENABLE();
      return TIM16;
      break;
    case DeviceId::Tim17:
      __HAL_RCC_TIM17_CLK_ENABLE();
      return TIM17;
      break;
    case DeviceId::Tim20:
      __HAL_RCC_TIM20_CLK_ENABLE();
      return TIM20;
      break;
    default:
      assert(false);
      break;
  }
}

void hal_timer_init(const DeviceConfigTimer* config) {
  assert(config->struct_size == sizeof(DeviceConfigTimer));
  int timer_index = (int)config->device_id - (int)DeviceId::Tim1;

  TIM_HandleTypeDef* tim_handle = &g_tim_handles[timer_index];
  TIM_TypeDef* instance = start_timer_clock(config->device_id);

  tim_handle->Instance = instance;
  tim_handle->Init.Prescaler = 0;
  tim_handle->Init.CounterMode = TIM_COUNTERMODE_UP;
  tim_handle->Init.Period = config->period;
  tim_handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim_handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Base_Init(tim_handle);
  HAL_TIM_Base_Start(tim_handle);
}

void hal_pwm_init(const DeviceConfigPwm* config) {
  assert(config->channel >= 1 && config->channel <= 4);

  int timer_index = (int)config->device_id - (int)DeviceId::Tim1;
  TIM_HandleTypeDef* tim_handle = &g_tim_handles[timer_index];

  int channel_index = config->channel - 1;

  uint32_t channel;
  switch (config->channel) {
    case 1:
      channel = TIM_CHANNEL_1;
      break;
    case 2:
      channel = TIM_CHANNEL_2;
      break;
    case 3:
      channel = TIM_CHANNEL_3;
      break;
    case 4:
      channel = TIM_CHANNEL_4;
      break;
  }

  g_pwm_channels[config->pwm_id].timer_id = timer_index;
  g_pwm_channels[config->pwm_id].channel_id = config->channel;
  g_pwm_channels[config->pwm_id].sign_pin = config->dir_pin;

  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(tim_handle, &sConfigOC, channel);

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  hal_gpio_init_pin_af(config->device_id, channel_index * 2, config->pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, channel_index * 2 + 1, config->n_pin, GPIO_InitStruct);

  if (config->dir_pin.port != 0xff) {
    GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    hal_gpio_init_pin(config->dir_pin, GPIO_InitStruct);
  }

  HAL_TIM_PWM_Start(tim_handle, channel);
}

void hal_encoder_init(const DeviceConfigEncoder* config) {
  int timer_index = (int)config->device_id - (int)DeviceId::Tim1;

  g_encoders[config->tim_index].timer_id = timer_index;
  g_encoders[config->tim_index].flags = config->flags;

  TIM_TypeDef* instance = start_timer_clock(config->device_id);

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM_HandleTypeDef* tim = &g_tim_handles[timer_index];
  tim->Instance = instance;
  tim->Init.Prescaler = 0;
  tim->Init.CounterMode = TIM_COUNTERMODE_UP;
  tim->Init.Period = config->period;
  tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim->Init.RepetitionCounter = 0;
  tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;  // TIM_TI1SELECTION_XORCOMBINATION
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  if (config->flags & EncoderFlags::HallMode) {
    HAL_TIM_ConfigTI1Input(tim, TIM_TI1SELECTION_XORCOMBINATION);
  } else {
    HAL_TIM_ConfigTI1Input(tim, TIM_TI1SELECTION_CH1);
  }

  HAL_TIM_Encoder_Init(tim, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(tim, &sMasterConfig);

  // config pins
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  hal_gpio_init_pin_af(config->device_id, 0, config->ch1_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 2, config->ch2_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 4, config->ch3_pin, GPIO_InitStruct);

  HAL_TIM_Encoder_Start(tim, TIM_CHANNEL_1);
}

}  // namespace platform
using namespace platform;
void pwm_set(int pwm_id, float value) {
  if (g_pwm_channels[pwm_id].timer_id == 0xff) {
    return;
  }

  auto htim = &g_tim_handles[g_pwm_channels[pwm_id].timer_id];
  auto descr = g_pwm_channels[pwm_id];

  uint32_t channel;
  switch (descr.channel_id) {
    case 1:
      channel = TIM_CHANNEL_1;
      break;
    case 2:
      channel = TIM_CHANNEL_2;
      break;
    case 3:
      channel = TIM_CHANNEL_3;
      break;
    case 4:
      channel = TIM_CHANNEL_4;
      break;
    default:
      return;
  }

  uint32_t pwm_raw = 0;
  bool dir_pin_state = false;

  if (value >= 0) {
    pwm_raw = static_cast<uint32_t>(value * htim->Init.Period);
    dir_pin_state = false;
  } else {
    pwm_raw = static_cast<uint32_t>(-value * htim->Init.Period);
    dir_pin_state = true;
  }

  if (pwm_raw > htim->Init.Period) {
    pwm_raw = htim->Init.Period;
  }
  __HAL_TIM_SET_COMPARE(htim, channel, pwm_raw);

  if (descr.sign_pin.port != 0xff) {
    hal_gpio_pin_set(descr.sign_pin, dir_pin_state);
  }
}

uint16_t encoder_get(int id) {
  auto descr = g_encoders[id];
  auto htim = &g_tim_handles[descr.timer_id];

  if (descr.flags & EncoderFlags::ReverseDirection)  // reverse direction flag
  {
    return htim->Instance->ARR - htim->Instance->CNT;
  } else {
    return htim->Instance->CNT;
  }
  return 0;
}

}  // namespace hal
}  // namespace goldobot

using namespace goldobot::hal;

#include <atomic>
uint16_t g_encoder_queue_left[16];
uint16_t g_encoder_queue_right[16];
std::atomic<int> g_encoder_queue_head{0};
std::atomic<int> g_encoder_queue_tail{0};

bool goldo_hal_read_encoders(uint16_t& left, uint16_t& right) {
  auto tail = g_encoder_queue_tail.load();
  if (tail == g_encoder_queue_head) {
    return false;
  }
  left = g_encoder_queue_left[tail];
  right = g_encoder_queue_right[tail];
  tail++;
  if (tail == 16) {
    tail = 0;
  }
  g_encoder_queue_tail = tail;
  return true;
}
void goldobot_hal_tim6_irq_handler() {
  if (!g_hal_initialized) return;
  uint16_t left = encoder_get(0);
  uint16_t right = encoder_get(1);

  g_encoder_queue_left[g_encoder_queue_head] = left;
  g_encoder_queue_right[g_encoder_queue_head] = right;

  auto head = g_encoder_queue_head.load() + 1;
  if (head == 16) head = 0;
  g_encoder_queue_head = head;
}
