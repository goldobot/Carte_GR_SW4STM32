#include "goldobot/platform/hal_gpio.hpp"

namespace goldobot { namespace platform {


void hal_gpio_init_pin(int port_index, LL_GPIO_InitTypeDef* init)
{
	switch(port_index)
	{
	case 0:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		LL_GPIO_Init(GPIOA, init);
		break;
	case 1:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		LL_GPIO_Init(GPIOB, init);
		break;
	case 2:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
		LL_GPIO_Init(GPIOC, init);
		break;
	case 3:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
		LL_GPIO_Init(GPIOD, init);
		break;
	case 4:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
		LL_GPIO_Init(GPIOE, init);
		break;
	case 5:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
		LL_GPIO_Init(GPIOF, init);
		break;
	}
}

} }; //namespace goldobot::platform
