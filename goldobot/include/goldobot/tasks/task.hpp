#pragma once
#include <atomic>
typedef void* TaskHandle_t;
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
		void delayTicks(unsigned ticks);
		void checkStateUpdate();

		std::atomic<State> m_state;
		State m_requested_state;

	private:
		static void vTaskFunction(void* thisptr);
		TaskHandle_t m_task_handle;
		TickType_t m_last_wake_time;
	};
}
