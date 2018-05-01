#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/core/circular_buffer.hpp"

#include <cstdint>

// To be removed once mutexes are abstracted
#include "FreeRTOS.h"
#include "task.h"

namespace goldobot
{
	class SimpleOdometry;
	class TrajectoryBuffer;


	class TrapezoidalSpeedProfile
	{
	public:
		TrapezoidalSpeedProfile();
		void update(float distance, float speed, float accel, float deccel);
		float begin_time();
		float end_time();
		void compute(float t, float* val, float* deriv, float* accel);

	private:
		float m_c0[4];
		float m_c1[4];
		float m_c2[4];
		float m_t[4];
	};





	class PropulsionController
	{
	public:
		enum class State
		{
			Inactive,
			Stopped,
			FollowTrajectory,
			PointTo,
			Reposition,
			EmergencyStop,
			Error
		};

		enum class Direction
		{
			Forward,
			Backward
		};

	public:
		PropulsionController(SimpleOdometry* odometry);
		State state() const;

		void update();

		float leftMotorPwm();
		float rightMotorPwm();

		bool executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration);
		bool executeRepositioning(float speed, Vector2D normal, float distance_to_center);
		RobotPose target_pose() const;


		Vector2D m_dbg_target_position_buffer[500];
		float m_dbg_target_yaw_buffer[500];
		RobotPose m_dbg_pose_buffer[500];

		int m_dbg_index;
		int m_dbg_counter;

	private:
		enum class CommandType : uint8_t
		{
			Trajectory,
			PointTo
		};
		struct Command
		{
			CommandType type;
			uint8_t num_points;
			Vector2D control_points[16];
			float speed;
			float acceleration;
			float decceleration;
		};

	private:
		SimpleOdometry* m_odometry;
		RobotPose m_pose;
		State m_state;

		float m_left_motor_pwm;
		float m_right_motor_pwm;

		float m_lateral_error;
		float m_longitudinal_error;
		float yaw_error;

		// Trajectory following controllers
		FeedForwardPIDController m_yaw_rate_pid;

		// Static positionning controllers
		FeedForwardPIDController m_translation_pid;
		FeedForwardPIDController m_yaw_pid;

		TrajectoryBuffer m_trajectory_buffer;
		TrapezoidalSpeedProfile m_speed_profile;

		// Parameters on current segment
		float m_current_parameter;
		uint32_t m_command_begin_time;
		uint32_t m_command_end_time;
		Direction m_direction;

		uint32_t m_time_base_ms;

		Vector2D m_target_position;
		float m_target_yaw;
		float m_target_speed;
		Vector2D m_lookahead_position;



		//! \brief compute motors pwm values when the robot is static. Use PID controllers on yaw and longitudinal position.
		void computeMotorsPwmStatic();

		//! \brief compute motors pwm values when the robot is moving. Use a pure pursuit algorithm and PID controllers on speed and yaw rate.
		void computeMotorsPwmMoving();

		//! \brief recompute current and lookahead positions.
		void updateTargetPositions();

		void computePositionError();

		// Initialize speed parameters for move command
		void initMoveCommand(float speed, float accel, float deccel);
	};
}
