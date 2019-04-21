#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trapezoidal_speed_profile.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/propulsion/controller_config.hpp"
#include "goldobot/core/circular_buffer.hpp"
#include "goldobot/messages.hpp"

#include <cstdint>

#include "FreeRTOS.h"
#include "semphr.h"


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
			YawRateSteps,
			PositionStaticSteps,
			YawSteps,
			PositionMovingSteps
		};

		enum class Error
		{
			None,
			EmergencyStop, // An emergency stop occurred during last command
			RobotBlocked, // Robot was blocked by an obstacle during last command
			TrackingError // The tracking error became too large during last command
		};

		enum class Direction
		{
			Forward,
			Backward
		};

	public:
		PropulsionController(SimpleOdometry* odometry);

		bool test;

		//! \brief Change state from Inactive to Stopped
		void enable();

		//! \brief Set the state to Inactive
		void disable();

		//! \brief Get current controller state
		State state() const;

		//! \brief Get current controller error code
		Error error() const;
		void clear_error();
		RobotPose target_pose() const;

		void update();

		float leftMotorPwm();
		float rightMotorPwm();

		//! \brief reset robot pose. Only works if state is Inactive or Stopped. Also change odometry.
		bool reset_pose(float x, float y, float yaw);

		bool executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration);
		bool executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center);
		bool executePointTo(Vector2D target, float yaw_rate, float accel, float deccel);
		bool executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel);

		//! \brief Emergency stop. Abort current PointTo of FollowTrajectory command and bring the robot to a stop.
		void emergency_stop();

		void executeTest(TestPattern patern);

		const PropulsionControllerConfig& config() const;
		void set_config(const PropulsionControllerConfig& config);

		messages::PropulsionTelemetry getTelemetry() const;
		/*< */
		messages::PropulsionTelemetryEx getTelemetryEx() const;


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

	private:
		PropulsionControllerConfig m_config;
		SimpleOdometry* m_odometry;
		RobotPose m_pose;
		State m_state;
		bool m_control_translation;
		bool m_control_yaw;
		bool m_control_speed;
		bool m_control_yaw_rate;
		Error m_error;
		TestPattern m_test_pattern;
		Vector2D m_test_initial_position;
		float m_test_initial_yaw;

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

		SemaphoreHandle_t m_mutex;


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

		void check_tracking_error();
		void update_test();

		void on_stopped_enter();
		void on_trajectory_exit();
		void on_rotation_exit();
		void on_reposition_exit();
		void on_test_exit();

		// Initialize speed parameters for move command
		void initMoveCommand(float speed, float accel, float deccel);
		void initRotationCommand(float delta_yaw, float speed, float accel, float deccel);
	};
}
