#include "goldobot/platform/hal_uart.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/platform/hal_dma.hpp"
#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_private.hpp"

#include "stm32f3xx_hal.h"

extern "C" {
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_usart.h"
}

extern "C" {
void goldobot_hal_uart_irq_handler(int uart_index);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart);
}

namespace goldobot {
namespace hal {
namespace platform {

UART_HandleTypeDef g_uart_handles[5];
IODevice* g_uart_io_devices[5];
PinID g_uart_txen_pins[5];

inline uint8_t get_uart_index(UART_HandleTypeDef* huart) {
  return static_cast<uint8_t>(huart - g_uart_handles);
}

}  // namespace platform
}  // namespace hal
}  // namespace goldobot

using namespace goldobot::hal::platform;

void goldobot_hal_uart_irq_handler(int uart_index) {
  auto& huart = g_uart_handles[uart_index];
 /* bool idle_detected{false};
  if (HAL_IS_BIT_SET(huart.Instance->CR1, UART_IT_IDLE) &&
      HAL_IS_BIT_SET(huart.Instance->ISR, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_FLAG(&huart, USART_ICR_IDLECF);
    idle_detected = true;
  }*/
  HAL_UART_IRQHandler(&g_uart_handles[uart_index]);
  /*if (idle_detected) {
    hal_callback_send_from_isr(HalCallback{DeviceType::Uart, static_cast<uint8_t>(uart_index), 3});
  }*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  auto io_device = g_uart_io_devices[uart_index];

  // Immediately send the queued next request to avoid overflow errors caused by task switching
  // delay
  if (io_device->rx_request_next.state == IORequestState::Pending) {
    io_device->rx_functions->start_request(&io_device->rx_request_next, uart_index);
  }
  io_device->rx_request_next.state = IORequestState::Busy;
  // Execute callback for just finished request
  if(uart_index == 2)
  {
	  int a = 1;
  }
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, uart_index, 0});
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  hal_gpio_pin_set(g_uart_txen_pins[uart_index], false);
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, uart_index, 1, 0});
};

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, uart_index, 2, 0});
}

namespace goldobot {
namespace hal {
namespace platform {

void hal_uart_callback(int uart_index, int callback_id) {
  auto io_device = g_uart_io_devices[uart_index];
  UART_HandleTypeDef* huart = &g_uart_handles[uart_index];

  switch (callback_id) {
    case 0:  // rx
    {
      // This callback should always be triggered just after an uart Rx completion interrupt
      auto req = &io_device->rx_request;

      auto next_state = io_device->rx_request_next.state.load();
      auto next_rx_ptr = io_device->rx_request_next.rx_ptr;
      auto next_size = io_device->rx_request_next.size;
      auto next_callback = io_device->rx_request_next.callback;
      io_device->rx_request_next.state.store(IORequestState::Ready);
      // We saved request_next and set state to Ready so that callback can enqueue a new request

      req->remaining = 0;
      req->state = IORequestState::Complete;

      if (req->callback) {
        req->callback(req, io_device);
      }

      if (next_state == IORequestState::Busy) {
        // The HAL receive call should have been made in the IRQ handler
        // Copy the new request to current slot and mark next request as ready
        io_device->rx_request.rx_ptr = next_rx_ptr;
        io_device->rx_request.size = next_size;
        io_device->rx_request.remaining = next_size;
        io_device->rx_request.callback = next_callback;
        io_device->rx_request.state = IORequestState::Busy;
      }
      return;
    }
    case 1:  // tx
    {
      auto req = &io_device->tx_request;
      req->remaining = huart->TxXferCount;
      req->state = IORequestState::Complete;
      if (req->callback) {
        req->callback(req, io_device);
      }
      return;
    } break;
    case 2:  // error
    {
      auto req = &io_device->rx_request;
      /*
      if (huart->ErrorCode & UART_FLAG_ORE) {

      }
      if (huart->RxState == HAL_UART_STATE_READY) {
      }*/
      HAL_UART_AbortReceive(huart);
      req->remaining = huart->RxXferCount;
      req->state = IORequestState::Complete;
      if (req->callback) {
        req->callback(req, io_device);
      }
      return;
    } break;
    case 3:  // rx idle interrupt
    {
    } break;
    default:
      break;
  }
}

bool uart_start_rx_request(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];

  hal_assert(req->state == IORequestState::Ready, HalEvent::UartRxStartErrorBadReqState);
  req->remaining = req->size;
  req->state = IORequestState::Busy;

  auto status = HAL_UART_Receive_IT(uart_handle, req->rx_ptr, req->size);
  if(status != HAL_OK)
  {
	  hal_trace_error(HalEvent::UartRxStartErrorHal);
	  return false;
  }
  return true;
}

bool uart_update_rx_request(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::Busy) {
    return false;
  }
  auto uart_handle = &g_uart_handles[device_index];
  req->remaining = uart_handle->RxXferCount;
  return true;
}

bool uart_start_tx_request(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];

  hal_assert(req->state == IORequestState::Ready, HalEvent::UartTxStartErrorBadReqState);
  req->remaining = req->size;
  req->state = IORequestState::Busy;

  hal_gpio_pin_set(g_uart_txen_pins[device_index], true);
  auto status = HAL_UART_Transmit_IT(uart_handle, req->tx_ptr, req->size);
  if(status != HAL_OK)
  {
	  hal_trace_error(HalEvent::UartTxStartErrorHal);
	  return false;
  }
  return true;
}

bool uart_update_tx_request(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::Busy) {
    return false;
  }
  auto uart_handle = &g_uart_handles[device_index];
  req->remaining = uart_handle->TxXferCount;
  return true;
}

bool uart_start_rx_request_dma(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];

  hal_assert(req->state == IORequestState::Ready, HalEvent::UartRxStartErrorBadReqState);
  req->remaining = req->size;
  req->state = IORequestState::Busy;


  auto status = HAL_UART_Receive_DMA(uart_handle, req->rx_ptr, req->size);
  if(status != HAL_OK)
  {
	  hal_trace_error(HalEvent::UartRxStartErrorHal);
	  return false;
  }
  return true;
}

bool uart_update_rx_request_dma(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];
  if (req->state != IORequestState::Busy) {
    return false;
  }
  req->remaining = uart_handle->hdmarx->Instance->CNDTR;
}

bool uart_start_tx_request_dma(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];

  hal_assert(req->state == IORequestState::Ready, HalEvent::UartTxStartErrorBadReqState);
  req->remaining = req->size;
  req->state = IORequestState::Busy;

  hal_gpio_pin_set(g_uart_txen_pins[device_index], true);
  auto status = HAL_UART_Transmit_DMA(uart_handle, req->tx_ptr, req->size);
  if(status != HAL_OK)
  {
	  hal_trace_error(HalEvent::UartTxStartErrorHal);
	  return false;
  }
  return true;
}

bool uart_update_tx_request_dma(IORequest* req, uint32_t device_index) {
  auto uart_handle = &g_uart_handles[device_index];

  if (req->state != IORequestState::Busy) {
    return false;
  }
  req->remaining = uart_handle->hdmatx->Instance->CNDTR;
  return true;
}

bool uart_rx_abort(IORequest* req, uint32_t device_index)
{
	auto uart_handle = &g_uart_handles[device_index];

	HAL_UART_AbortReceive(uart_handle);
	return true;
}

bool uart_tx_abort(IORequest* req, uint32_t device_index)
{
	auto uart_handle = &g_uart_handles[device_index];

	HAL_UART_AbortTransmit(uart_handle);
	return true;
}

IODeviceFunctions g_uart_device_rx_functions = {&uart_start_rx_request, &uart_update_rx_request, &uart_rx_abort};

IODeviceFunctions g_uart_device_tx_functions = {&uart_start_tx_request, &uart_update_tx_request, &uart_tx_abort};

IODeviceFunctions g_uart_device_rx_functions_dma = {&uart_start_rx_request_dma,
                                                    &uart_update_rx_request_dma, &uart_rx_abort};

IODeviceFunctions g_uart_device_tx_functions_dma = {&uart_start_tx_request_dma,
                                                    &uart_update_tx_request_dma, &uart_tx_abort};

void hal_usart_init(IODevice* device, const IODeviceConfigUart* config) {
  int uart_index = (int)config->device_id - (int)DeviceId::Usart1;

  IRQn_Type irq_n;
  UART_HandleTypeDef* uart_handle = &g_uart_handles[uart_index];
  USART_TypeDef* uart_instance;

  switch (config->device_id) {
    case DeviceId::Usart1:
      uart_instance = USART1;
      irq_n = USART1_IRQn;
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
      break;

    case DeviceId::Usart2:
      uart_instance = USART2;
      irq_n = USART2_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
      break;
    case DeviceId::Usart3:
      uart_instance = USART3;
      irq_n = USART3_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
      break;
    case DeviceId::Uart4:
      uart_instance = UART4;
      irq_n = UART4_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
      break;
    case DeviceId::Uart5:
      uart_instance = UART5;
      irq_n = UART5_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
      break;
    default:
      break;
  }

  g_uart_io_devices[uart_index] = device;
  device->device_index = uart_index;

  if (config->io_flags & IODeviceFlags::RxDma) {
    device->rx_functions = &g_uart_device_rx_functions_dma;
  } else {
    device->rx_functions = &g_uart_device_rx_functions;
  }

  if (config->io_flags & IODeviceFlags::TxDma) {
    device->tx_functions = &g_uart_device_tx_functions_dma;
  } else {
    device->tx_functions = &g_uart_device_tx_functions;
  }

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

  hal_gpio_init_pin_af(config->device_id, 0, config->rx_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 1, config->tx_pin, GPIO_InitStruct);

  g_uart_txen_pins[uart_index] = config->txen_pin;
  if (config->txen_pin.port != 0xff) {
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    hal_gpio_init_pin(config->txen_pin, GPIO_InitStruct);
  }

  /* USART2 interrupt Init */
  HAL_NVIC_SetPriority(irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
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
  assert(status == HAL_OK);

  /*if (config->io_flags & IODeviceFlags::IdleInterrupt) {
    __HAL_UART_ENABLE_IT(uart_handle, UART_IT_IDLE);
  }*/

  if (config->io_flags & IODeviceFlags::RxDma) {
    DMA_InitTypeDef DMAInit;
    DMAInit.Direction = DMA_PERIPH_TO_MEMORY;
    DMAInit.PeriphInc = DMA_PINC_DISABLE;
    DMAInit.MemInc = DMA_MINC_ENABLE;
    DMAInit.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMAInit.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMAInit.Mode = DMA_NORMAL;
    DMAInit.Priority = DMA_PRIORITY_LOW;

    auto hdma = hal_dma_init_device(config->device_id, 0, DMAInit);
    __HAL_LINKDMA(uart_handle, hdmarx, *hdma);
  }

  if (config->io_flags & IODeviceFlags::TxDma) {
    DMA_InitTypeDef DMAInit;
    DMAInit.Direction = DMA_MEMORY_TO_PERIPH;
    DMAInit.PeriphInc = DMA_PINC_DISABLE;
    DMAInit.MemInc = DMA_MINC_ENABLE;
    DMAInit.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMAInit.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMAInit.Mode = DMA_NORMAL;
    DMAInit.Priority = DMA_PRIORITY_LOW;

    auto hdma = hal_dma_init_device(config->device_id, 1, DMAInit);
    __HAL_LINKDMA(uart_handle, hdmatx, *hdma);
  }
}

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
