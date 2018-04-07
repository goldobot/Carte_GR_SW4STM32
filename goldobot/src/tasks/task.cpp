#include "goldobot/tasks/task.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

Task::Task():
		m_state(Task::Uninitialized)
{}

Task::~Task()
{}

Task::State Task::state() const
{
	return m_state;
}

void Task::start()
{
	if(m_state == Stopped)
	{
		m_state = Running;
	}
}

void Task::stop()
{
	if(m_state == Running)
	{
		m_state = Stopped;
	}
}

void Task::init()
{
	// Check for double initilaization.
	if(m_state != Uninitialized)
	{
		return;
	}

	m_state = Stopped;

	// Create propulsion thread. This is executed every 1 ms on highest priority.
	xTaskCreate(
			&Task::vTaskFunction,
			name(),
			200,// Stack size
			this,
			configMAX_PRIORITIES - 1,
			&m_task_handle);
}

void Task::delayTicks(unsigned ticks)
{
	vTaskDelayUntil(&m_last_wake_time, ticks);
}

void Task::vTaskFunction(void* thisptr)
{
	reinterpret_cast<Task*>(thisptr)->m_last_wake_time = xTaskGetTickCount();
	reinterpret_cast<Task*>(thisptr)->taskFunction();
}