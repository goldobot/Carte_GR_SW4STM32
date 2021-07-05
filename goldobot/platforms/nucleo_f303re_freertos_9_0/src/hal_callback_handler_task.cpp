#include "goldobot/platform/hal_private.hpp"

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <atomic>

extern "C" {
void goldobot_hal_exti_irq_handler();
}

namespace goldobot {
namespace hal {
namespace platform {
TaskHandle_t g_hal_callback_handler_task_handle;
}
}  // namespace hal
}  // namespace goldobot

using namespace goldobot::hal::platform;

void goldobot_hal_exti_irq_handler() {
  bool is_swi = (EXTI->SWIER & 0x00000001u) != 0;
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  if (is_swi) {
    BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(g_hal_callback_handler_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

namespace goldobot {
namespace hal {
namespace platform {

constexpr unsigned c_hal_callback_queue_size = 32;

std::atomic<std::int_fast8_t> g_hal_callback_write_head{0};
std::atomic<std::int_fast8_t> g_hal_callback_read_head{0};
std::atomic<std::int_fast8_t> g_hal_callback_tail{0};

HalCallback g_hal_callback_queue[c_hal_callback_queue_size];

void hal_callback_handler_task_function(void* thisptr) {
  while (true) {
    xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);

    while (g_hal_callback_tail != g_hal_callback_read_head) {
      const auto& callback = g_hal_callback_queue[g_hal_callback_tail];

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
      auto new_tail = g_hal_callback_tail + 1;
      if (new_tail == c_hal_callback_queue_size) {
        new_tail = 0;
      }
      g_hal_callback_tail = new_tail;
    }
  }
}

void hal_callback_handler_task_start() {
  xTaskCreate(hal_callback_handler_task_function, "hal_callback_handler", 128, nullptr,
              (configMAX_PRIORITIES - 1), &g_hal_callback_handler_task_handle);

  HAL_NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  EXTI->IMR |= GPIO_PIN_0;
};

void hal_callback_send(const HalCallback& callback) {
  std::int_fast8_t head = g_hal_callback_write_head.load(std::memory_order_relaxed);

  // atomically allocate a slot in the callbacks queue
  // compare_exchange_weak write the next value of head if it was not modified by a higher priority
  // interrupt if it was modified, retry with new value
  while (!g_hal_callback_write_head.compare_exchange_weak(
      head, head + 1 != c_hal_callback_queue_size ? head + 1 : 0, std::memory_order_release,
      std::memory_order_relaxed)) {
  };
  g_hal_callback_queue[head] = callback;

  // atomically advance read head to signal the callback data has been written
  head = g_hal_callback_read_head.load(std::memory_order_relaxed);
  while (!g_hal_callback_read_head.compare_exchange_weak(
      head, head + 1 != c_hal_callback_queue_size ? head + 1 : 0, std::memory_order_release,
      std::memory_order_relaxed)) {
  };
  __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
}

void hal_callback_send_from_isr(const HalCallback& callback) { hal_callback_send(callback); }

}  // namespace platform
}  // namespace hal
};  // namespace goldobot
