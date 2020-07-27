#pragma once
#include <atomic>
#include <thread>
#include <chrono>

typedef uint32_t TickType_t;

namespace goldobot
{
	enum class TaskId
	{
		Propulsion,
		Log,
		MatchTimer,
		Strategy
	};

	class Task
	{
	public:
		enum State
		{
			Uninitialized,
			Stopped,
			Running
		};

	public:
		virtual ~Task();
		virtual const char* name() const = 0;

		State state() const;

		void init();
		void start();
		void stop();

	protected:
		Task();
		virtual void taskFunction() = 0;

		void set_priority(unsigned prio);
		void delay(unsigned ticks);
		void delay_periodic(unsigned ticks);
		void checkStateUpdate();

		std::atomic<State> m_state;
		State m_requested_state;

	private:
		std::thread m_thread;
		std::chrono::time_point<std::chrono::steady_clock> m_last_wake_time;
	};
}
