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
	enum class Side : uint16_t
	{
		Unknown=0,
		Green=1,
		Orange=2
	};

	enum class OpCode : uint8_t
	{
		Invalid=0,
		SetPose=1,
		Rotation=2,
		PointTo=3,
		Trajectory=4,
		Reposition=5,
		ArmsGoToPosition=6,
		ArmsExecuteSequence=7,
		Delay=8,

	};

	struct Command
	{
		OpCode opcode;
		uint8_t blocking;
		union
		{
			uint8_t padding[8];
			uint16_t delay_ms;
			struct
			{
				uint16_t pose_id;
				int16_t angle_deg;
			} set_pose;
			struct
			{
				int16_t angle;
				uint8_t speed_settings;
				uint8_t reserved;
			} rotation;
			struct
			{
				uint16_t begin_idx;
				uint8_t num_points;
				uint8_t speed_settings;
			} trajectory;
			struct
			{
				int16_t angle;
				int16_t distance;
			} reposition;
			struct
			{
				uint8_t arm_id;
				uint8_t reserved;
				uint16_t seq_or_pos_id;
			} arms;
		};
	};

	struct Sequence
	{
		uint16_t begin_idx;
		uint16_t end_idx;
	};

	struct SpeedSettings
	{
		float speed;
		float acceleration;
		float decceleration;
		float yaw_rate;
		float yaw_acceleration;
		float yaw_decceleration;
		float reposition_speed;
	};

	class MainTask : public Task
	{
	public:
		MainTask();
		const char* name() const override;

		// Number of seconds before end of match
		int remainingMatchTime();
		void preMatchBegin();
		void preMatchStep();
		void matchBegin();
		void matchStep();

		void matchSelectNextObjective();

		bool push_message(uint16_t message_type, const unsigned char* buffer, size_t size);

		void execute_sequence(int id);
		bool _execute_command(const Command& cmd);
		bool _current_command_finished(const Command& cmd);

		void sequence_step();



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
		void on_msg_dbg_arms_go_to_position();
		void on_msg_dbg_arms_execute_sequence();

		void on_msg_dbg_robot_set_command();
		void on_msg_dbg_robot_set_point();
		void on_msg_dbg_robot_set_sequence();
		void on_msg_dbg_robot_execute_sequence();
		void on_msg_dbg_robot_set_trajectory_point();

		void taskFunction() override;

		State m_match_state;
		Side m_side;
		uint32_t m_start_of_match_time;
		TrajectoryPlanner m_trajectory_planner;
		uint16_t m_current_trajectory_index;
		uint16_t m_current_objective;
		SemaphoreHandle_t m_dbg_message_queue_mutex;
		MessageQueue m_dbg_message_queue;
		unsigned char m_dbg_message_queue_buffer[512];

		Vector2D m_waypoints[128];
		uint8_t m_trajectory_points[256];
		Command m_commands[512];
		Sequence m_sequences[32];
		SpeedSettings m_speed_settings[8];

		bool m_sequence_active;
		bool m_wait_current_cmd;
		uint16_t m_current_sequence_id;
		uint16_t m_current_command_id;
		uint32_t m_delay_finished_ts;
	};
}
