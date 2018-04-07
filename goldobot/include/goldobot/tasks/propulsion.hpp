#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/command_queue.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"

#include <cstdint>

namespace goldobot
{
	class PropulsionTask : public Task
	{
	public:
		PropulsionTask();
		const char* name() const override;

		SimpleOdometry& odometry();

	private:
		SimpleOdometry m_odometry;
		uint16_t m_encoder_left;
		uint16_t m_encoders_right;

		void doStep();
		void taskFunction() override;
	};
}
