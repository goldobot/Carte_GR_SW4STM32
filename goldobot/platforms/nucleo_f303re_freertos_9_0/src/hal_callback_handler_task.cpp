#include "FreeRTOS.h"
#include "goldobot/platform/hal_private.hpp"
#include "semphr.h"
#include "task.h"

namespace goldobot {
namespace hal {
namespace platform {

QueueHandle_t g_hal_callback_queue;
TaskHandle_t g_hal_callback_handler_task_handle;

void hal_callback_handler_task_function(void* thisptr) {
  while (true) {
    HalCallback callback;
    if (xQueueReceive(g_hal_callback_queue, &callback, portMAX_DELAY) == pdTRUE) {
      switch (callback.device_type) {
        case DeviceType::Uart:
          hal_uart_callback(callback.device_index, callback.callback_index);
          break;
        case DeviceType::IODevice:
          hal_iodevice_callback(callback.device_index, callback.callback_index);
          break;
        default:
          break;
      }
    }
  }
}

void hal_callback_handler_task_start() {
  g_hal_callback_queue = xQueueCreate(16, sizeof(HalCallback));

  xTaskCreate(hal_callback_handler_task_function, "hal_callback_handler", 128, nullptr,
              (configMAX_PRIORITIES - 1), nullptr);
};

void hal_callback_send(const HalCallback& callback) {
  BaseType_t xHigherPriorityTaskWoken;
  xQueueSend(g_hal_callback_queue, &callback, portMAX_DELAY);
}

void hal_callback_send_from_isr(const HalCallback& callback) {
  BaseType_t xHigherPriorityTaskWoken;
  xQueueSendFromISR(g_hal_callback_queue, &callback, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

}  // namespace platform
}  // namespace hal
};  // namespace goldobot
