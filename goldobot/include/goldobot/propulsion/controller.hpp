#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trapezoidal_speed_profile.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/core/circular_buffer.hpp"

#include <cstdint>


namespace goldobot
{
	class SimpleOdometry;
	class TrajectoryBuffer;

	struct PropulsionConfiguration
	{
		float lookahead_distance;
		float lookahead_time;
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
			Error,
			Test
		};

		enum TestPattern
		{
			SpeedSteps,
			PositionSteps
		};

		enum class Error
		{
			EmergencyStop, // An emergency stop occurred during last command
			RobotBlocked,
			TrackingError // The tracking error became too large during last command
		};

		enum class Direction
		{
			Forward,
			Backward
		};

	public:
		PropulsionController(SimpleOdometry* odometry);

		State state() const;
		RobotPose target_pose() const;

		void update();

		float leftMotorPwm();
		float rightMotorPwm();

		bool executeTrajectory(Vector2D* points, int num_points, Direction direction, float speed, float acceleration, float decceleration);
		bool executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center);
		bool executePointTo(Vector2D target, float yaw_rate, float accel, float deccel);
		bool executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel);

		void executeTest(TestPattern patern);

		void set_speed_feedforward(float ff);
		void set_speed_kp(float kp);
		void set_speed_kd(float kd);
		void set_speed_ki(float ki);

		void set_translation_kp(float kp);
		void set_translation_kd(float kd);
		void set_translation_ki(float ki);


		Vector2D m_dbg_target_position_buffer[500];
		float m_dbg_target_yaw_buffer[500];
		float m_dbg_target_speed_buffer[500];
		float m_dbg_left_pwm_buffer[500];
		float m_dbg_right_pwm_buffer[500];
		RobotPose m_dbg_pose_buffer[500];

		int m_dbg_index;
		int m_dbg_counter;

	//private:
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

	//private:
		SimpleOdometry* m_odometry;
		RobotPose m_pose;
		State m_state;
		TestPattern m_test_pattern;

		float m_left_motor_pwm;
		float m_right_motor_pwm;
		float m_pwm_limit;

		

		// PID controllers
		// Inner loop speed control PIDs
		PIDController m_yaw_rate_pid;
		PIDController m_speed_pid;

		// Outer loop position control PIDs
		PIDController m_translation_pid;
		PIDController m_yaw_pid;


		TrajectoryBuffer m_trajectory_buffer;
		float m_begin_yaw; // yaw at beginning of current PointTo command
		TrapezoidalSpeedProfile m_speed_profile;

		// Parameters on current segment
		float m_current_parameter;
		uint32_t m_command_begin_time;
		uint32_t m_command_end_time;
		Direction m_direction;

		uint32_t m_time_base_ms;

		// Targets
		Vector2D m_target_position;
		float m_target_yaw;
		float m_target_speed;
		float m_target_yaw_rate;
		Vector2D m_lookahead_position;

		// Errors
		float m_lateral_error;
		float m_longitudinal_error;
		float m_yaw_error;
		float m_speed_error;
		float m_yaw_rate_error;

		bool m_reposition_hit;
		// Line equation for border on which to reposition
		// Should take into account robot size in config
		Vector2D m_reposition_border_normal;
		float m_reposition_border_distance;


		//! \brief compute motors pwm values when the robot is static. Use PID controllers on yaw and longitudinal position
		void updateMotorsPwm();

		//! \brief compute motors pwm in test mode
		void updateMotorsPwmTest();

		//! \brief Update current and lookahead positions in TrajectoryFollowing mode
		void updateTargetPositions();

		//! \brief Update target yaw and yaw rate in PointTo mode
		void updateTargetYaw();

		void updateReposition();

		void repositionReconfigureOdometry();

		bool detectBlockage();

		// Initialize speed parameters for move command
		void initMoveCommand(float speed, float accel, float deccel);
		void initRotationCommand(float delta_yaw, float speed, float accel, float deccel);
	};
}
