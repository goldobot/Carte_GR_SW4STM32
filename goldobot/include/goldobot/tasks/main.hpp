#pragma once
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/trajectory_planner.hpp"

#include <cstdint>

namespace goldobot
{
	enum class StartingSide : uint16_t
	{
		Unknown=0,
		Green=1,
		Orange=2
	};
	class MainTask : public Task
	{
	public:
		MainTask();
		const char* name() const override;

		void setStartingSide(StartingSide side);
		// Number of seconds before end of match
		int remainingMatchTime();
		void preMatchBegin();
		void preMatchStep();
		void matchBegin();
		void matchStep();

		void matchSelectNextObjective();

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
		StartingSide m_starting_side;
		uint32_t m_start_of_match_time;
		TrajectoryPlanner m_trajectory_planner;
		uint16_t m_current_trajectory_index;
		uint16_t m_current_objective;
	};
}
