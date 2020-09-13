#pragma once
#include "goldobot/platform/task.hpp"
#include "goldobot/platform/message_queue.hpp"


namespace goldobot
{
	class DebugTask : public Task
	{
	public:
		DebugTask();
		const char* name() const override;



	private:
		MessageQueue m_message_queue;
		unsigned char m_message_queue_buffer[256];

		void doStep();
		void processMessage();

		void taskFunction() override;
	};
}
