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

const IRQn_Type c_uart_irq_numbers[5] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn,
                                         UART5_IRQn};

IORequest* g_uart_rx_io_requests[5];
IORequest* g_uart_tx_io_requests[5];

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
using namespace goldobot::hal;

void goldobot_hal_uart_irq_handler(int uart_index) {
  // auto& huart = g_uart_handles[uart_index];
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

void goldbot_uart_chain_request(IODevice* io_device, int uart_index, IORequest* req) {
  if (req == nullptr) {
    // should not happen
    return;
  }
  req->remaining = 0;
  if (req->callback) {
    bool status = req->callback(req, IORequestStatus::Success);
    if (status) {
      if (req->rx_ptr != nullptr) {
        io_device->rx_functions->start_request(req, uart_index);
        return;
      } else if (req->tx_ptr) {
        io_device->tx_functions->start_request(req, uart_index);

        return;
      }
    }
  }
  hal_callback_send_from_isr(HalCallback{DeviceType::Uart, uart_index, 0});
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  auto io_device = g_uart_io_devices[uart_index];
  auto req = g_uart_rx_io_requests[uart_index];
  g_uart_rx_io_requests[uart_index] = nullptr;

  goldbot_uart_chain_request(io_device, uart_index, req);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  hal_gpio_pin_set(g_uart_txen_pins[uart_index], false);
  auto io_device = g_uart_io_devices[uart_index];
  auto req = g_uart_tx_io_requests[uart_index];
  g_uart_tx_io_requests[uart_index] = nullptr;

  goldbot_uart_chain_request(io_device, uart_index, req);
};

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  uint8_t uart_index = get_uart_index(huart);
  hal_callback_send(HalCallback{DeviceType::Uart, uart_index, 1, 0});
}

namespace goldobot {
namespace hal {
namespace platform {

void hal_uart_callback(int uart_index, int callback_id) {
  auto irqn = c_uart_irq_numbers[uart_index];
  UART_HandleTypeDef* huart = &g_uart_handles[uart_index];

  // end of transfer for execute
  if (callback_id == 0) {
    auto io_device = g_uart_io_devices[uart_index];
    xSemaphoreGive(io_device->rx_semaphore);
  }
  // uart transfer error
  if (callback_id == 1) {
    // disable uart interrupt
    IRQLock lock(irqn);
    HAL_UART_Abort(huart);

    {
      auto req = g_uart_rx_io_requests[uart_index];
      g_uart_rx_io_requests[uart_index] = nullptr;

      if (req && req->callback) {
        req->callback(req, IORequestStatus::Error);
      }
    }
    {
      auto req = g_uart_tx_io_requests[uart_index];
      g_uart_tx_io_requests[uart_index] = nullptr;

      if (req && req->callback) {
        req->callback(req, IORequestStatus::Error);
      }
    }
  }
}

bool uart_start_rx_request(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  if (g_uart_rx_io_requests[device_index] != nullptr) {
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }

  g_uart_rx_io_requests[device_index] = req;

  req->remaining = req->size;

  auto status = HAL_UART_Receive_IT(uart_handle, req->rx_ptr, req->size);

  if (status != HAL_OK) {
    hal_trace_error(HalEvent::UartRxStartErrorHal);
    g_uart_rx_io_requests[device_index] = nullptr;
    return false;
  }
  return true;
}

bool uart_update_rx_request(IORequest* req, uint32_t device_index) {
  // get uart handle and irq number
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_rx_io_requests[device_index];
  if (req) {
    req->remaining = uart_handle->RxXferCount;
    req->callback(req, IORequestStatus::Update);
  }
  return true;
}

bool uart_start_tx_request(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  if (g_uart_tx_io_requests[device_index] != nullptr) {
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }

  g_uart_tx_io_requests[device_index] = req;

  req->remaining = req->size;

  hal_gpio_pin_set(g_uart_txen_pins[device_index], true);
  auto status = HAL_UART_Transmit_IT(uart_handle, req->tx_ptr, req->size);

  if (status != HAL_OK) {
    hal_gpio_pin_set(g_uart_txen_pins[device_index], false);
    g_uart_tx_io_requests[device_index] = nullptr;
    hal_trace_error(HalEvent::UartTxStartErrorHal);
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }
  return true;
}

bool uart_update_tx_request(IORequest* req, uint32_t device_index) {
  // get uart handle and irq number
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_tx_io_requests[device_index];
  if (req) {
    req->remaining = uart_handle->TxXferCount;
    req->callback(req, IORequestStatus::Update);
  }
  return true;
}

bool uart_start_rx_request_dma(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  // fail if trying to start a request while another one is already in progess
  if (g_uart_rx_io_requests[device_index]) {
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    // reenable interrupt
    return false;
  }
  g_uart_rx_io_requests[device_index] = req;

  req->remaining = req->size;

  auto status = HAL_UART_Receive_DMA(uart_handle, req->rx_ptr, req->size);
  if (status != HAL_OK) {
    hal_trace_error(HalEvent::UartRxStartErrorHal);
    g_uart_rx_io_requests[device_index] = nullptr;
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }
  return true;
}

bool uart_update_rx_request_dma(IORequest* req, uint32_t device_index) {
  // get uart handle and irq number
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_rx_io_requests[device_index];
  if (req) {
    req->remaining = uart_handle->hdmarx->Instance->CNDTR;
    req->callback(req, IORequestStatus::Update);
  }
  return true;
}

bool uart_start_tx_request_dma(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  if (g_uart_tx_io_requests[device_index] != nullptr) {
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }

  g_uart_tx_io_requests[device_index] = req;

  req->remaining = req->size;
  // req->state = IORequestState::Busy;

  hal_gpio_pin_set(g_uart_txen_pins[device_index], true);
  auto status = HAL_UART_Transmit_DMA(uart_handle, req->tx_ptr, req->size);
  if (status != HAL_OK) {
    hal_gpio_pin_set(g_uart_txen_pins[device_index], false);
    hal_trace_error(HalEvent::UartTxStartErrorHal);
    g_uart_tx_io_requests[device_index] = nullptr;
    if (req->callback) {
      req->callback(req, IORequestStatus::Error);
    }
    return false;
  }
  return true;
}

bool uart_update_tx_request_dma(IORequest* req, uint32_t device_index) {
  // get uart handle and irq number
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_tx_io_requests[device_index];
  if (req) {
    req->remaining = uart_handle->hdmatx->Instance->CNDTR;
    req->callback(req, IORequestStatus::Update);
  }
  return true;
}

bool uart_rx_abort(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_rx_io_requests[device_index] = req;
  g_uart_rx_io_requests[device_index] = nullptr;

  HAL_UART_AbortReceive(uart_handle);

  if (uart_handle->hdmarx != NULL) {
    uart_handle->hdmarx->XferAbortCallback = NULL;
    HAL_DMA_Abort(uart_handle->hdmarx);
  };

  if (req && req->callback) {
    req->callback(req, IORequestStatus::Abort);
  }
  return true;
}

bool uart_tx_abort(IORequest* req, uint32_t device_index) {
  auto irqn = c_uart_irq_numbers[device_index];
  auto uart_handle = &g_uart_handles[device_index];

  // disable uart interrupt
  IRQLock lock(irqn);

  req = g_uart_tx_io_requests[device_index];
  g_uart_tx_io_requests[device_index] = nullptr;

  HAL_UART_AbortTransmit(uart_handle);
  if (uart_handle->hdmatx != NULL) {
    uart_handle->hdmatx->XferAbortCallback = NULL;
    HAL_DMA_Abort(uart_handle->hdmatx);
  };

  if (req && req->callback) {
    req->callback(req, IORequestStatus::Abort);
  }
  return true;
}

IODeviceFunctions g_uart_device_rx_functions = {&uart_start_rx_request, &uart_update_rx_request,
                                                &uart_rx_abort};

IODeviceFunctions g_uart_device_tx_functions = {&uart_start_tx_request, &uart_update_tx_request,
                                                &uart_tx_abort};

IODeviceFunctions g_uart_device_rx_functions_dma = {&uart_start_rx_request_dma,
                                                    &uart_update_rx_request_dma, &uart_rx_abort};

IODeviceFunctions g_uart_device_tx_functions_dma = {&uart_start_tx_request_dma,
                                                    &uart_update_tx_request_dma, &uart_tx_abort};

void hal_usart_init(IODevice* device, const IODeviceConfigUart* config) {
  int uart_index = (int)config->device_id - (int)DeviceId::Usart1;

  g_uart_rx_io_requests[uart_index] = nullptr;
  g_uart_tx_io_requests[uart_index] = nullptr;

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
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Alternate = 0;
    hal_gpio_init_pin(config->txen_pin, GPIO_InitStruct);
  }

  /* USART2 interrupt Init */
  HAL_NVIC_SetPriority(irq_n, 4, 0);
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
