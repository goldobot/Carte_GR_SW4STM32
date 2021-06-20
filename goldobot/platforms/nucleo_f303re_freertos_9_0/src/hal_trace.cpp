#include "stm32f3xx_hal.h"
#include "core_cm4.h"

#include "FreeRTOS.h"
#include "task.h"

#include "goldobot/platform/hal_private.hpp"

extern "C" {
void goldo_trace_task_switched_out(void* task);
void goldo_trace_task_switched_in(void* task);
void goldo_trace_task_create(void* task);
void goldo_trace_init(void);
}


namespace goldobot {
namespace hal {
namespace platform {
// temporary
  // hal error tracing
  struct HalEventData
  {
  	uint32_t cycnt;
  	uint8_t task_number;
  	uint8_t interrupt_number;
  	HalEvent event;
  };

  constexpr size_t c_hal_trace_buffer_size = 128;
  HalEventData g_hal_trace_buffer[c_hal_trace_buffer_size];
  const HalEventData* g_hal_trace_buffer_end = g_hal_trace_buffer + c_hal_trace_buffer_size;

  HalEventData* g_hal_trace_head{g_hal_trace_buffer};

  void hal_trace_push(const HalEventData& evt)
  {
	  HalEventData* head = g_hal_trace_head;
	  auto new_head = head + 1;
	  if(new_head == g_hal_trace_buffer_end)
	  {
		  new_head = g_hal_trace_buffer;
	  }
	  // /todo: should use atomics to ensure no other interrupt has pushed an event
	  g_hal_trace_head = new_head;

	  *head = evt;
  }


  void hal_trace_event(HalEvent event_code)
  {
  	HalEventData evt;
  	evt.cycnt = DWT->CYCCNT;
  	uint8_t interrupt_number = __get_IPSR() & 0xff;
  	uint8_t task_number = -1;
  	if(interrupt_number == 0)
  	{
  		// we are in task mode, get the current task from FreeRTOS
  		task_number = uxTaskGetTaskNumber(xTaskGetCurrentTaskHandle()) & 0xff;
  	}
  	evt.interrupt_number = interrupt_number;
  	evt.task_number = (uint8_t)task_number;
  	evt.event = event_code;

  	hal_trace_push(evt);
  }

  void hal_trace_error(HalEvent event_code)
    {
	  hal_trace_event(event_code);
    }
}
}
}



