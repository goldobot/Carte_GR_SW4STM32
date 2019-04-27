#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/core/message_queue.hpp"

#include <cstdint>

namespace goldobot
{
	class PropulsionTask : public Task
	{
	public:
		PropulsionTask();
		const char* name() const override;

		SimpleOdometry& odometry();
		PropulsionController& controller();

	private:
		SimpleOdometry m_odometry;
		PropulsionController m_controller;
		uint16_t m_encoder_left;
		uint16_t m_encoders_right;
		uint16_t m_telemetry_counter;
		PropulsionController::State m_previous_state{PropulsionController::State::Inactive};

		MessageQueue m_message_queue;
		unsigned char m_message_queue_buffer[512];

		void doStep();
		void processMessage();
		void taskFunction() override;

		void onMsgExecuteTrajectory();
	};
}
