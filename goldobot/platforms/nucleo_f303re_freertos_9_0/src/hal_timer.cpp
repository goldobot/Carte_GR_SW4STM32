#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_gpio.hpp"

extern "C"
{
	#include "stm32f3xx_hal_tim.h"
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"
}



namespace goldobot { namespace platform {

TIM_HandleTypeDef g_tim_handles[10];

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

static

void hal_encoder_init(IODevice* device, const DeviceConfigEncoder* config)
{
	IRQn_Type irq_n;
	uint32_t gpio_alternate;
	TIM_TypeDef* tim_instance;

	switch(config->tim_index)
	{
	case 0:
		tim_instance = TIM1;
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
		break;
	case 3:
		tim_instance = TIM4;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		break;
	default:
		break;
	}


	goldobot_hal_s_usart_io_devices[config->tim_index] = device;


	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->ch1_pin);
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	//GPIO_InitStruct.Alternate = gpio_alternate;
    //todo: write tables to find correct alternate function for timer pins
	hal_gpio_init_pin(config->ch1_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->ch2_pin);
	hal_gpio_init_pin(config->ch2_port, &GPIO_InitStruct);


  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM_HandleTypeDef* tim = &g_tim_handles[config->tim_index];
  tim->Instance = tim_instance;
  tim->Init.Prescaler = 0;
  tim->Init.CounterMode = TIM_COUNTERMODE_UP;
  tim->Init.Period = config->period;
  tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim->Init.RepetitionCounter = 0;
  tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(tim, &sConfig) != HAL_OK)
  {
	//Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(tim, &sMasterConfig) != HAL_OK)
  {
	//Error_Handler();
  }
  HAL_TIM_Encoder_Start(tim, TIM_CHANNEL_1);
}

} } // namespace goldobot::platform
