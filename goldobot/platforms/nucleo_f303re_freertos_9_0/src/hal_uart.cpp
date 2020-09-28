#include "goldobot/platform/hal_uart.hpp"

#include "goldobot/platform/hal_dma.hpp"
#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_private.hpp"
#include "stm32f3xx_hal.h"
extern "C" {
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_ll_bus.h"
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
int g_hal_uart_current_uart_index;

UART_HandleTypeDef g_hal_uart_handles[5];
IODevice* g_hal_uart_io_devices[5];
PinID g_hal_uart_tx_pins[5];

}  // namespace platform
}  // namespace hal
}  // namespace goldobot

using namespace goldobot::hal::platform;

void goldobot_hal_uart_irq_handler(int uart_index) {
  g_hal_uart_current_uart_index = uart_index;
  HAL_UART_IRQHandler(&g_hal_uart_handles[uart_index]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, g_hal_uart_current_uart_index, 0, 0});
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, g_hal_uart_current_uart_index, 1, 0});
};

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, g_hal_uart_current_uart_index, 2, 0});
}

namespace goldobot {
namespace hal {
namespace platform {

void hal_uart_callback(int uart_index, int callback_id) {
  auto io_device = g_hal_uart_io_devices[uart_index];
  UART_HandleTypeDef* huart = &g_hal_uart_handles[uart_index];

  switch (callback_id) {
    case 0:  // rx
    {
      auto req = &io_device->rx_request;
      req->remaining = huart->RxXferCount;
      assert(req->remaining == 0);
      req->state = IORequestState::RxComplete;
      if (req->callback) {
        req->callback(req, io_device);
      }
      return;
    }
    case 1:  // tx
    {
      auto req = &io_device->tx_request;
      req->remaining = huart->TxXferCount;
      req->state = IORequestState::TxComplete;
      if (req->callback) {
        req->callback(req, io_device);
      }
      return;
    } break;
    case 2:  // error
    {
      auto req = &io_device->rx_request;
      if (huart->RxState == HAL_UART_STATE_READY) {
        req->remaining = huart->RxXferCount;
        req->state = IORequestState::RxComplete;
        if (req->callback) {
          req->callback(req, io_device);
        }
      }
      return;
    } break;
    default:
      break;
  }
}

void uart_start_rx_request(IORequest* req, uint32_t device_index) {
  assert(req->state == IORequestState::Ready);
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = req->size;
  req->state = IORequestState::RxBusy;
  hal_gpio_pin_set(g_hal_uart_tx_pins[device_index], true);
  auto status = HAL_UART_Receive_IT(uart_handle, req->rx_ptr, req->size);
  assert(status == HAL_OK);
}

void uart_update_rx_request(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::RxBusy) {
    return;
  }
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = uart_handle->RxXferCount;
}

void uart_start_tx_request(IORequest* req, uint32_t device_index) {
  assert(req->state == IORequestState::Ready);
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = req->size;
  req->state = IORequestState::TxBusy;
  auto status = HAL_UART_Transmit_IT(uart_handle, req->tx_ptr, req->size);
  assert(status == HAL_OK);
}

void uart_update_tx_request(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::TxBusy) {
    return;
  }
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = uart_handle->TxXferCount;
}

void uart_start_rx_request_dma(IORequest* req, uint32_t device_index) {
  assert(req->state == IORequestState::Ready);
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = req->size;
  req->state = IORequestState::RxBusy;
  auto status = HAL_UART_Receive_DMA(uart_handle, req->rx_ptr, req->size);
  assert(status == HAL_OK);
}

void uart_update_rx_request_dma(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::RxBusy) {
    return;
  }
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = uart_handle->hdmarx->Instance->CNDTR;
}

void uart_start_tx_request_dma(IORequest* req, uint32_t device_index) {
  assert(req->state == IORequestState::Ready);
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = req->size;
  req->state = IORequestState::TxBusy;
  auto status = HAL_UART_Transmit_DMA(uart_handle, req->tx_ptr, req->size);
  assert(status == HAL_OK);
}

void uart_update_tx_request_dma(IORequest* req, uint32_t device_index) {
  if (req->state != IORequestState::TxBusy) {
    return;
  }
  auto uart_handle = &g_hal_uart_handles[device_index];
  req->remaining = uart_handle->hdmatx->Instance->CNDTR;
}

IODeviceFunctions g_uart_device_rx_functions = {&uart_start_rx_request, &uart_update_rx_request, 0};

IODeviceFunctions g_uart_device_tx_functions = {&uart_start_tx_request, &uart_update_tx_request, 0};

IODeviceFunctions g_uart_device_rx_functions_dma = {&uart_start_rx_request_dma,
                                                    &uart_update_rx_request_dma, 0};

IODeviceFunctions g_uart_device_tx_functions_dma = {&uart_start_tx_request_dma,
                                                    &uart_update_tx_request_dma, 0};

void hal_usart_init(IODevice* device, const IODeviceConfigUart* config) {
  int uart_index = (int)config->device_id - (int)DeviceId::Usart1;

  IRQn_Type irq_n;
  UART_HandleTypeDef* uart_handle = &g_hal_uart_handles[uart_index];
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

  g_hal_uart_io_devices[uart_index] = device;
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

  hal_gpio_init_pin_af(config->device_id, 0, config->rx_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 1, config->tx_pin, GPIO_InitStruct);

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
