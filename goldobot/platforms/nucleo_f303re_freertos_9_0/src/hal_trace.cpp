#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"
#include "core_cm4.h"

extern "C"
{
  void goldo_trace_task_switched_out(void* task);
  void goldo_trace_task_switched_in(void* task);
  void goldo_trace_task_create(void* task);
  void goldo_trace_init(void);
}

uint8_t trace_task_tag = 0;

struct TraceEvent
{
	uint8_t task_tag;
	uint8_t event_type;
	uint16_t reserved;
	uint32_t cycle_counter;
};

TraceEvent g_trace_events[256];
int g_trace_event_head=0;

void goldo_trace_init()
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	ITM->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CTRL |= DWT_EXCCNT_EXCCNT_Msk;
}

void goldo_trace_task_create(void* task)
{
	uint32_t cycle_counter = DWT->CYCCNT;
	int task_tag = trace_task_tag;
	trace_task_tag++;
	vTaskSetApplicationTaskTag((TaskHandle_t)task, (TaskHookFunction_t)task_tag);

	g_trace_events[g_trace_event_head].task_tag = task_tag;
	g_trace_events[g_trace_event_head].event_type = 0;
	g_trace_events[g_trace_event_head].reserved = 0;
	g_trace_events[g_trace_event_head].cycle_counter = cycle_counter;
	g_trace_event_head++;
	if(g_trace_event_head == sizeof(g_trace_events)) g_trace_event_head = 0;

}

void goldo_trace_task_switched_in(void* task)
{
	uint32_t cycle_counter = DWT->CYCCNT;
	int task_tag = (int)xTaskGetApplicationTaskTagFromISR((TaskHandle_t)task);

	g_trace_events[g_trace_event_head].task_tag = task_tag;
	g_trace_events[g_trace_event_head].event_type = 1;
	g_trace_events[g_trace_event_head].reserved = 0;
	g_trace_events[g_trace_event_head].cycle_counter = cycle_counter;
	g_trace_event_head++;
	if(g_trace_event_head == sizeof(g_trace_events)) g_trace_event_head = 0;
}

void goldo_trace_task_switched_out(void* task)
{
	uint32_t cycle_counter = DWT->CYCCNT;
	int task_tag = (int)xTaskGetApplicationTaskTagFromISR((TaskHandle_t)task);

	g_trace_events[g_trace_event_head].task_tag = task_tag;
	g_trace_events[g_trace_event_head].event_type = 2;
	g_trace_events[g_trace_event_head].reserved = 0;
	g_trace_events[g_trace_event_head].cycle_counter = cycle_counter;
	g_trace_event_head++;
	if(g_trace_event_head == sizeof(g_trace_events)) g_trace_event_head = 0;
}
