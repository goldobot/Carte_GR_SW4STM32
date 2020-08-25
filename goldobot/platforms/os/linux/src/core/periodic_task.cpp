#include "goldobot/platform/periodic_task.hpp"

using namespace goldobot;

Task::Task():
		m_state()
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
	if (m_state != Uninitialized)
	{
		return;
	}

	m_state = Stopped;

	// Create propulsion thread. This is executed every 1 ms on highest priority.
	m_last_wake_time = std::chrono::steady_clock::now();
	m_thread = std::thread{&Task::taskFunction, this};
}

void Task::delay_periodic(unsigned ticks)
{
	auto tick_count = std::chrono::steady_clock::now();
	if(tick_count - m_last_wake_time > std::chrono::milliseconds(ticks))
	{
		m_last_wake_time = tick_count;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(ticks) + m_last_wake_time - tick_count);
	m_last_wake_time = std::chrono::steady_clock::now();
}

void Task::checkStateUpdate()
{

}
