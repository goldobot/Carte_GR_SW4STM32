#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_gpio.hpp"

extern "C"
{
	#include "stm32f3xx_hal_uart.h"
	#include "stm32f3xx_ll_gpio.h"
	#include "stm32f3xx_ll_bus.h"

    void goldobot_hal_uart_irq_handler(int uart_index);
	extern void* goldobot_hal_s_usart_io_devices[5];
}


void* goldobot_hal_s_usart_io_devices[5];

namespace goldobot { namespace platform {

UART_HandleTypeDef g_uart_handles[5];

} }; // namespace goldobot::platform



extern "C"
{
	void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
	{
		for(unsigned i = 0; i < 5; i++)
		{
			if(huart == &goldobot::platform::g_uart_handles[i])
			{
				auto device = static_cast<goldobot::platform::IODevice*>(goldobot_hal_s_usart_io_devices[i]);
				auto req = &device->tx_request;
				req->remaining = huart->TxXferCount;
				req->state = goldobot::platform::IORequestState::TxComplete;
				if(req->callback)
				{
					req->callback(req, device);
				}
				return;
			}
		}

	/*	if (huart!=&huart3) {
			BaseType_t xHigherPriorityTaskWoken;
			xSemaphoreGiveFromISR(s_uart_semaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}*/
	};

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
	{
		for(unsigned i = 0; i < 5; i++)
		{
			if(huart == &goldobot::platform::g_uart_handles[i])
			{
				auto device = static_cast<goldobot::platform::IODevice*>(goldobot_hal_s_usart_io_devices[i]);
				auto req = &device->rx_request;
				req->remaining = huart->RxXferCount;
				req->state = goldobot::platform::IORequestState::RxComplete;
				if(req->callback)
				{
					req->callback(req, device);
				}
				return;
			}
		}
	}
}

void goldobot_hal_uart_irq_handler(int uart_index)
{
	HAL_UART_IRQHandler(&goldobot::platform::g_uart_handles[uart_index]);
}

namespace goldobot { namespace platform {


void uart_start_rx_request(IORequest* req, void* device_handle)
{
	auto uart_handle = static_cast<UART_HandleTypeDef*>(device_handle);
	req->remaining = req->size;
	req->state = IORequestState::RxBusy;
	HAL_UART_Receive_IT(uart_handle, req->ptr, req->size);
}

void uart_update_rx_request(IORequest* req, void* device_handle)
{
	if(req->state != IORequestState::RxBusy)
	{
		return;
	}
	auto uart_handle = static_cast<UART_HandleTypeDef*>(device_handle);
	req->remaining = uart_handle->RxXferCount;
}

void uart_start_tx_request(IORequest* req, void* device_handle)
{
	auto uart_handle = static_cast<UART_HandleTypeDef*>(device_handle);
	req->remaining = req->size;
	req->state = IORequestState::TxBusy;
	HAL_UART_Transmit_IT(uart_handle, req->ptr, req->size);
}

void uart_update_tx_request(IORequest* req, void* device_handle)
{
	if(req->state != IORequestState::TxBusy)
	{
		return;
	}
	auto uart_handle = static_cast<UART_HandleTypeDef*>(device_handle);
	req->remaining = uart_handle->TxXferCount;
}

IODeviceFunctions g_uart_device_functions = {
		&uart_start_rx_request,
		&uart_update_rx_request,
		0,
		&uart_start_tx_request,
		&uart_update_tx_request,
		0
};


void hal_usart_init(IODevice* device, const goldobot::IODeviceConfigUART* config)
{
	IRQn_Type irq_n;
	uint32_t gpio_alternate;
	UART_HandleTypeDef* uart_handle = &g_uart_handles[config->uart_index];
	USART_TypeDef* uart_instance;

	switch(config->uart_index)
	{
	case 0:
		uart_instance = USART1;
		irq_n = USART1_IRQn;
		gpio_alternate = GPIO_AF7_USART1;
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		break;

	case 1:
		uart_instance = USART2;
		irq_n = USART2_IRQn;
		gpio_alternate = GPIO_AF7_USART2;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		break;
	case 2:
		uart_instance = USART3;
		irq_n = USART3_IRQn;
		gpio_alternate = GPIO_AF7_USART3;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		break;
	case 3:
		uart_instance = UART4;
		irq_n = UART4_IRQn;
		gpio_alternate = GPIO_AF5_UART4;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		break;
	case 4:
		uart_instance = UART5;
		irq_n = UART5_IRQn;
		gpio_alternate = GPIO_AF5_UART5;
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
		break;
	default:
		break;
	}


	goldobot_hal_s_usart_io_devices[config->uart_index] = device;
	device->device_handle = uart_handle;
	device->functions = &g_uart_device_functions;


	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->rx_pin);
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = gpio_alternate;

	hal_gpio_init_pin(config->rx_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = (uint16_t)( 1U << config->tx_pin);
	hal_gpio_init_pin(config->tx_port, &GPIO_InitStruct);


	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(irq_n, 5, 0);
	HAL_NVIC_EnableIRQ(irq_n);

	uart_handle->Instance = uart_instance;
	uart_handle->Init.BaudRate = config->baudrate;
	uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	uart_handle->Init.StopBits = UART_STOPBITS_1;
	uart_handle->Init.Parity = UART_PARITY_NONE;
	uart_handle->Init.Mode = UART_MODE_TX_RX;
	uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
	uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	auto status = HAL_UART_Init(uart_handle);
}

} } //namespace goldobot::platform
