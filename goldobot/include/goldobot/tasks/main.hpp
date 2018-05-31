#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/message_types.hpp"
#include "goldobot/tasks/task.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/trajectory_planner.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"


namespace goldobot
{
	enum class StartingSide : uint16_t
	{
		Unknown=0,
		Green=1,
		Orange=2
	};

	enum class OpCode : uint16_t
	{
		SetPose=1,
		ExecuteTrajectory=2,
		PointTo=3,
	};

	struct Command
	{
		uint8_t begin_index;
		uint8_t end_index;
		float speed;
		float acceleration;
		float decceleration;
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

		bool push_message(uint16_t message_type, const unsigned char* buffer, size_t size);



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

		void pop_message(unsigned char* buffer, size_t size);
		void process_messages();
		void process_message(CommMessageType message_type, uint16_t message_size);

		void on_msg_dbg_execute_trajectory();
		void on_msg_dbg_arms_set_pose();
		void on_msg_dbg_arms_set_command();
		void on_msg_dbg_arms_set_sequences(uint16_t message_size);
		void on_msg_dbg_arms_execute_sequence();

		void taskFunction() override;

		State m_match_state;
		StartingSide m_starting_side;
		uint32_t m_start_of_match_time;
		TrajectoryPlanner m_trajectory_planner;
		uint16_t m_current_trajectory_index;
		uint16_t m_current_objective;
		SemaphoreHandle_t m_dbg_message_queue_mutex;
		MessageQueue m_dbg_message_queue;
		unsigned char m_dbg_message_queue_buffer[512];

		Vector2D m_waypoints[64];
		uint8_t m_trajectories[128];
		Command m_commands[32];
	};
}
