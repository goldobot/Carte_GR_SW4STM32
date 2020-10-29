#include "goldobot/platform/task.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

Task::Task() : m_state(Task::Uninitialized) {}

Task::~Task() {}

Task::State Task::state() const { return m_state; }

void Task::start() {
  if (m_state == Stopped) {
    m_state = Running;
  }
}

void Task::stop() {
  if (m_state == Running) {
    m_state = Stopped;
  }
}

void Task::init(size_t stack_size) {
  // Check for double initilaization.
  if (m_state != Uninitialized) {
    return;
  }

  if(stack_size == 0)
  {
	  stack_size = 512;
  }

  m_state = Stopped;

  // Create propulsion thread. This is executed every 1 ms on highest priority.
  auto status = xTaskCreate(&Task::vTaskFunction, name(),
		  	  stack_size,  // Stack size
              this, configMAX_PRIORITIES - 1, &m_task_handle);
  if(status != pdPASS)
  {
	  __asm("BKPT");
  }
}

void Task::set_priority(unsigned prio) { vTaskPrioritySet(m_task_handle, prio); }

void Task::delay(unsigned ticks) { vTaskDelay(ticks); }

void Task::delay_periodic(unsigned ticks) {
  auto tick_count = xTaskGetTickCount();
  if (tick_count - m_last_wake_time > ticks) {
    m_last_wake_time = tick_count;
  }
  vTaskDelayUntil(&m_last_wake_time, ticks);
}

void Task::vTaskFunction(void* thisptr) {
  reinterpret_cast<Task*>(thisptr)->m_last_wake_time = xTaskGetTickCount();
  reinterpret_cast<Task*>(thisptr)->taskFunction();
}

void Task::checkStateUpdate() {}
