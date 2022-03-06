#include "stm32f3xx_hal.h"
#include "core_cm4.h"

#include "FreeRTOS.h"
#include "task.h"

#include "goldobot/platform/hal_private.hpp"

#include <atomic>

extern "C" {
void goldo_trace_task_switched_out(void* task);
void goldo_trace_task_switched_in(void* task);
void goldo_trace_task_create(void* task, unsigned tcb_number);
void goldo_trace_init(void);
}

namespace goldobot {
namespace hal {
namespace platform {
// temporary
// hal error tracing
struct HalEventData {
  uint32_t cycnt;
  uint8_t task_number;
  uint8_t interrupt_number;
  uint8_t event;
  uint8_t payload;
};

constexpr size_t c_hal_trace_buffer_size = 64;
HalEventData g_hal_trace_buffer[c_hal_trace_buffer_size];
const HalEventData* g_hal_trace_buffer_end = g_hal_trace_buffer + c_hal_trace_buffer_size;
uint8_t g_hal_trace_task_number{0};

std::atomic<HalEventData*> g_hal_trace_head{g_hal_trace_buffer};

HalEventData* hal_trace_push() {
  HalEventData* head = g_hal_trace_head;
  while(true) {
	  auto new_head = head + 1;
	  if (new_head == g_hal_trace_buffer_end) {
		new_head = g_hal_trace_buffer;
	  }
	  if(g_hal_trace_head.compare_exchange_weak(head, new_head))
		  return head;
  }
}

void hal_trace_event(HalEvent event_code, uint8_t payload) {
  auto cycnt = DWT->CYCCNT;
  uint8_t interrupt_number = __get_IPSR() & 0xff;

  HalEventData& evt = *hal_trace_push();

  evt.cycnt = cycnt;
  evt.interrupt_number = interrupt_number;
  evt.task_number = (uint8_t)(uxTaskGetTaskNumber(xTaskGetCurrentTaskHandle()) & 0xff);
  evt.event = (uint8_t)event_code;
  evt.payload = payload;
}

}  // namespace platform
std::tuple<const char*, size_t> dbg_get_trace_buffer() {
	return {(const char*)platform::g_hal_trace_buffer, sizeof(platform::g_hal_trace_buffer)};
}
}  // namespace hal
}  // namespace goldobot

// kernel trace functions implementation
void goldo_trace_task_switched_in(void* task) {
	using namespace goldobot::hal::platform;
	hal_trace_event(HalEvent::OSTaskSwitchedIn);
}

void goldo_trace_task_switched_out(void* task) {
	using namespace goldobot::hal::platform;
	hal_trace_event(HalEvent::OSTaskSwitchedOut);
}

void goldo_trace_task_create(void* task, unsigned tcb_number) {
	using namespace goldobot::hal::platform;
	vTaskSetTaskNumber((TaskHandle_t)task, tcb_number);
}
