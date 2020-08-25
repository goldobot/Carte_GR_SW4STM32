#include "goldobot/platform/hal_private.hpp"

extern "C"
{
	#include "stm32f3xx_ll_usart.h"
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"

	void goldobot_hal_usart_irq_handler(void* device);
	extern void* goldobot_hal_s_usart_io_devices[5];
}


void* goldobot_hal_s_usart_io_devices[5] = {
	0,
	0,
	0,
	0,
	0
};

void goldobot_hal_uart_enable_receive_it(goldobot::IODevice* device)
{
	auto usart = static_cast<USART_TypeDef*>(device->device_handle);
	LL_USART_EnableIT_ERROR(usart);
	LL_USART_EnableIT_RXNE(usart);
}

void goldobot_hal_init_gpio_pin(int port_index, LL_GPIO_InitTypeDef* init)
{
	switch(port_index)
	{
	case 0:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		LL_GPIO_Init(GPIOA, init);
		break;
	}
}

void goldobot_hal_usart_init(goldobot::IODevice* device, const goldobot::IODeviceConfigUART* config)
{
	IRQn_Type irq_n;
	uint32_t gpio_alternate;

	switch(config->uart_index)
	{
	case 0:
		device->device_handle = USART1;
		irq_n = USART1_IRQn;
		gpio_alternate = GPIO_AF7_USART1;
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		break;

	case 1:
		device->device_handle = USART2;
		irq_n = USART2_IRQn;
		gpio_alternate = GPIO_AF7_USART2;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		break;
	case 2:
		device->device_handle = USART3;
		irq_n = USART3_IRQn;
		gpio_alternate = GPIO_AF7_USART3;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		break;
	case 3:
		device->device_handle = UART4;
		irq_n = UART4_IRQn;
		gpio_alternate = GPIO_AF5_UART4;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		break;
	case 4:
		device->device_handle = UART5;
		irq_n = UART5_IRQn;
		gpio_alternate = GPIO_AF5_UART5;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
		break;
	default:
		break;
	}


	goldobot_hal_s_usart_io_devices[config->uart_index] = device;
	auto usart = static_cast<USART_TypeDef*>(device->device_handle);


	// Must be refactored



	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->rx_pin);
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = gpio_alternate;

	goldobot_hal_init_gpio_pin(config->rx_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->tx_pin);
	goldobot_hal_init_gpio_pin(config->tx_port, &GPIO_InitStruct);


	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(irq_n, 5, 0);
	HAL_NVIC_EnableIRQ(irq_n);


	LL_USART_InitTypeDef usart_init;
	usart_init.BaudRate            = config->baudrate;
	usart_init.DataWidth           = LL_USART_DATAWIDTH_8B;
	usart_init.StopBits            = LL_USART_STOPBITS_1;
	usart_init.Parity              = LL_USART_PARITY_NONE ;
	usart_init.TransferDirection   = LL_USART_DIRECTION_TX_RX;
	usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	usart_init.OverSampling        = LL_USART_OVERSAMPLING_16;

	LL_USART_Disable(usart);
	auto status = LL_USART_Init(usart, &usart_init);

	if(status == SUCCESS)
	{
		LL_USART_Enable(usart);
		goldobot_hal_uart_enable_receive_it(device);
	}
}

void goldobot_hal_usart_irq_handler(void* handle)
{
	auto device = static_cast<goldobot::IODevice*>(handle);
	auto usart = static_cast<USART_TypeDef*>(device->device_handle);
	uint32_t isrflags   = READ_REG(usart->ISR);
	uint32_t cr1its     = READ_REG(usart->CR1);

	uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));

	if(errorflags  == 0U)
	{
		// Receive byte
		if(((isrflags & USART_ISR_RXNE) != 0U)
        && ((cr1its & USART_CR1_RXNEIE) != 0U)) {
			device->rx_queue.push_byte(usart->RDR);
		}

	}

	// Clear overrun flag
	if(errorflags & USART_ICR_ORECF)
	{
		usart->ICR = USART_ICR_ORECF;
	}

	// Clear framing error
	if(errorflags & USART_ISR_FE)
	{
		usart->ICR = USART_ISR_FE;
	}

	// Clear noise error
	if(errorflags & USART_ISR_NE)
	{
		usart->ICR = USART_ISR_NE;
	}

	if(((isrflags & USART_ISR_TC) != 0U)
	&& ((cr1its & USART_CR1_TCIE) != 0U))
	{
		int b = 1;
	}
}
