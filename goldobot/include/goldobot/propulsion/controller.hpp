#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/pid_controller.hpp"
#include "goldobot/core/trapezoidal_speed_profile.hpp"
#include "goldobot/core/trajectory_buffer.hpp"
#include "goldobot/propulsion/controller_config.hpp"
#include "goldobot/core/circular_buffer.hpp"
#include "goldobot/messages.hpp"
#include "goldobot/propulsion/low_level_controller.hpp"

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
		enum class State : uint8_t
		{
			Inactive,
			Stopped,
			FollowTrajectory,
			Rotate,
			Reposition,
			ManualControl,
			EmergencyStop,
			Error,
			LowLevelTest
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

		bool test{true};

		//! \brief Change state from Inactive to Stopped
		void enable();

		//! \brief Set the state to Inactive
		void disable();

		//! \brief Get current controller state
		State state() const;

		//! \brief Get current controller error code
		Error error() const;
		void clearError();
		RobotPose targetPose() const;

		void update();

		float leftMotorPwm();
		float rightMotorPwm();

		//! \brief reset robot pose. Only works if state is Inactive or Stopped. Also change odometry.
		bool resetPose(float x, float y, float yaw);

		bool executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration);
		bool executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center);
		bool executePointTo(Vector2D target, float yaw_rate, float accel, float deccel);
		bool executeMoveTo(Vector2D target, float yaw_rate, float accel, float deccel);
		bool executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel);
		bool executeTranslation(float distance, float speed, float accel, float deccel);

		//! \brief Emergency stop. Abort current PointTo of FollowTrajectory command and bring the robot to a stop.
		void emergencyStop();

		const PropulsionControllerConfig& config() const;
		void setConfig(const PropulsionControllerConfig& config);

		messages::PropulsionTelemetry getTelemetry() const;
		/*< */
		messages::PropulsionTelemetryEx getTelemetryEx() const;


	private:
		SimpleOdometry* m_odometry;
		PropulsionControllerConfig m_config;
		LowLevelController m_low_level_controller;
		RobotPose m_current_pose;
		RobotPose m_target_pose;


		State m_state{State::Inactive};
		Error m_error{Error::None};

		float m_left_motor_pwm{0};
		float m_right_motor_pwm{0};
		float m_pwm_limit{1.0f};

		TrajectoryBuffer m_trajectory_buffer;
		float m_begin_yaw; // yaw at beginning of current PointTo command
		TrapezoidalSpeedProfile m_speed_profile;

		// Parameters on current segment
		float m_current_parameter;
		uint32_t m_command_begin_time;
		uint32_t m_command_end_time;
		Direction m_direction;

		uint32_t m_time_base_ms{0};


		Vector2D m_lookahead_position;

		bool m_reposition_hit;
		// Line equation for border on which to reposition
		// Should take into account robot size in config
		Vector2D m_reposition_border_normal;
		float m_reposition_border_distance;

		//! \brief compute motors pwm values when the robot is static. Use PID controllers on yaw and longitudinal position
		void updateMotorsPwm();

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
