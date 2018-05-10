#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"

#include <cstdint>

namespace goldobot
{
	class MainTask : public Task
	{
	public:
		MainTask();
		const char* name() const override;

		// Number of seconds before end of match
		int remainingMatchTime();

		void matchStep();

	private:
		enum class State
		{
			Idle, // Initial state
			Debug, // Debug mode
			PreMatch, // Pre match repositioning sequence
			WaitForStartOfMatch, // Ready for match, waiting for start signal
			Match, // Match
			PostMatch // Match finished
		};

		void taskFunction() override;

		State m_match_state;
		uint32_t m_start_of_match_time;
	};
}
