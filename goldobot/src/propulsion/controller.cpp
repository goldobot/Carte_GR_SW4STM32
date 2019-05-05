#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
using namespace goldobot;

#include <algorithm>







PropulsionController::PropulsionController(SimpleOdometry* odometry):
		m_odometry(odometry)
{
}

void PropulsionController::enable()
{
	if(m_state != State::Inactive)
	{
		return;
	}
	m_current_pose = m_odometry->pose();
	m_target_pose = m_current_pose;
	on_stopped_enter();
}

void PropulsionController::disable()
{
	m_error = Error::None;
	m_state = State::Inactive;
	m_left_motor_pwm = 0;
	m_right_motor_pwm = 0;
}

PropulsionController::State PropulsionController::state() const
{
	return m_state;
}

PropulsionController::Error PropulsionController::error() const
{
	return m_error;
}

const PropulsionControllerConfig& PropulsionController::config() const
{
	return m_config;
}

void PropulsionController::setConfig(const PropulsionControllerConfig& config)
{
	m_config = config;
}

void PropulsionController::clearError()
{
	m_state = State::Stopped;
	m_error = Error::None;
	m_target_pose = m_current_pose;
}

void PropulsionController::emergencyStop()
{
	if(m_state == State::FollowTrajectory || m_state == State::Rotate)
	{
		m_state = State::EmergencyStop;
	}
}

void PropulsionController::update()
{
	m_current_pose = m_odometry->pose();
	switch(m_state)
	{
	case State::Inactive:
		break;
	case State::Stopped:
		updateMotorsPwm();
		if(fabsf(m_low_level_controller.m_longi_error) > 0.1f)
		{
			m_state = State::Error;
		}
		break;
	case State::FollowTrajectory:
		{
			updateTargetPositions();
			if(m_time_base_ms >= m_command_end_time)
			{
				on_trajectory_exit();
				on_stopped_enter();
			}
		}
		break;
	case State::Rotate:
		{
			updateTargetYaw();
			if (m_time_base_ms >= m_command_end_time)
			{
				on_rotation_exit();
				on_stopped_enter();
			}
		}
		break;
	case State::Reposition:
		{
			m_pwm_limit = m_config.reposition_pwm_limit;
			updateReposition();
			// Check position error
			if(m_time_base_ms >= m_command_end_time)
			{
				on_reposition_exit();
				on_stopped_enter();
			}
		}
		break;
	case State::EmergencyStop:
		{
			m_left_motor_pwm = 0;
			m_right_motor_pwm = 0;
			if (fabsf(m_current_pose.speed) < 0.01 && fabsf(m_current_pose.yaw_rate) < 0.1)
			{
				m_state = State::Error;
				m_error = Error::EmergencyStop;
			}
		}
		break;
	case State::Error:
		m_left_motor_pwm = 0;
		m_right_motor_pwm = 0;
		break;
	case State::LowLevelTest:
		{
			m_pwm_limit = 1;
			update_test();

			if (m_time_base_ms >= m_command_end_time)
			{
				on_test_exit();
			}
		}
		break;
	}

	if(m_state != State::Inactive && m_state != State::Error)
	{
		updateMotorsPwm();
	}
	// Update time base
	m_time_base_ms++;
}


float PropulsionController::leftMotorPwm()
{
	return m_left_motor_pwm;
}

float PropulsionController::PropulsionController::rightMotorPwm()
{
	return m_right_motor_pwm;
}

RobotPose PropulsionController::targetPose() const
{
	return m_target_pose;
}


void PropulsionController::update_test()
{
}

void PropulsionController::updateTargetPositions()
{
	// Compute current distance on trajectory target
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, speed, accel;
	m_speed_profile.compute(t,&parameter,&speed,&accel);
	parameter = std::min(parameter, m_trajectory_buffer.max_parameter());

	// Compute position of lookahead point in front of current position
	float lookahead_distance = m_config.lookahead_distance + fabsf(speed) * m_config.lookahead_time;
	float lookahead_parameter = parameter + lookahead_distance;

	// Compute target position
	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_pose.position = target_point.position;



	// Compute position of lookahead point
	// Extend the trajectory past last point if necessary
	if(lookahead_parameter <= m_trajectory_buffer.max_parameter())
	{
		m_lookahead_position = m_trajectory_buffer.compute_point(lookahead_parameter).position;
	} else
	{
		auto end_point = m_trajectory_buffer.compute_point(m_trajectory_buffer.max_parameter());
		m_lookahead_position.x = end_point.position.x + lookahead_distance * end_point.tangent.x;
		m_lookahead_position.y = end_point.position.y + lookahead_distance * end_point.tangent.y;
	}

	// Compute speed. project trajectory speed on robot axis, to get correct value during curves
	// This also has the consequence of slowing in curves
	// Current robot frame direction
	float ux = cosf(m_current_pose.yaw);
	float uy = sinf(m_current_pose.yaw);

	m_target_pose.speed = speed * (ux * target_point.tangent.x + uy * target_point.tangent.y);

	// Pure pursuit computation, update target yaw rate
	float diff_x = m_lookahead_position.x - m_current_pose.position.x;
	float diff_y = m_lookahead_position.y - m_current_pose.position.y;

	float rel_x = diff_x * ux + diff_y * uy;
	float rel_y = -diff_x * uy + diff_y * ux;

	float curvature = 2 * rel_y / (rel_x*rel_x + rel_y * rel_y);
	m_target_pose.yaw_rate = m_current_pose.speed * curvature;
	m_target_pose.yaw = m_current_pose.yaw;
}

void PropulsionController::updateTargetYaw()
{
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, accel;
	m_speed_profile.compute(t, &parameter, &m_target_pose.yaw_rate, &accel);
	m_target_pose.yaw = clampAngle(m_begin_yaw + parameter);
}

void PropulsionController::updateMotorsPwm()
{
	// Execute low level control
	m_low_level_controller.update(m_current_pose, m_target_pose);
	m_left_motor_pwm = m_low_level_controller.m_left_motor_pwm;
	m_right_motor_pwm = m_low_level_controller.m_right_motor_pwm;

	// Clamp outputs
	m_left_motor_pwm = clamp(m_left_motor_pwm, -m_pwm_limit, m_pwm_limit);
	m_right_motor_pwm = clamp(m_right_motor_pwm, -m_pwm_limit, m_pwm_limit);
}

void PropulsionController::updateReposition()
{
	float ux = cosf(m_target_pose.yaw);
	float uy = sinf(m_target_pose.yaw);

	m_target_pose.position.x += ux * m_target_pose.speed * 1e-3;
	m_target_pose.position.y += uy * m_target_pose.speed * 1e-3;

	if(fabs(m_low_level_controller.m_longi_error) > 0.06 && ! m_reposition_hit)
	{
		m_reposition_hit = true;
		m_command_end_time = m_time_base_ms + 500;
	}
};

void PropulsionController::on_stopped_enter()
{
	m_state = State::Stopped;
	m_low_level_controller.m_longi_control_level = 2;
	m_low_level_controller.m_yaw_control_level = 2;

	m_target_pose.speed = 0;
	m_target_pose.yaw_rate = 0;

	//m_translation_pid.set_config(m_config.translation_pid_config);
	m_pwm_limit = m_config.static_pwm_limit;
}

void PropulsionController::on_trajectory_exit()
{
	float parameter = m_trajectory_buffer.max_parameter();
	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_pose.position = target_point.position;
	m_target_pose.yaw = atan2f(target_point.tangent.y, target_point.tangent.x);

	if(m_direction == Direction::Backward)
	{
		m_target_pose.yaw = clampAngle(m_target_pose.yaw + M_PI);
	}
}

void PropulsionController::on_rotation_exit()
{
	// \todo cleanup
	float t = (m_command_end_time - m_command_begin_time) * 1e-3f;
	float parameter, accel;
	m_speed_profile.compute(t, &parameter, &m_target_pose.yaw_rate, &accel);
	m_target_pose.yaw = clampAngle(m_begin_yaw + parameter);
}

void PropulsionController::on_reposition_exit()
{
	if(m_reposition_hit)
	{
		repositionReconfigureOdometry();
		m_low_level_controller.reset();
	}
}

void PropulsionController::on_test_exit()
{
	m_low_level_controller.reset();

	m_target_pose.position = m_current_pose.position;
	m_target_pose.yaw = m_current_pose.yaw;
}

void PropulsionController::repositionReconfigureOdometry()
{
	m_odometry->measureLineNormal(m_reposition_border_normal, m_reposition_border_distance);
	auto pose = m_odometry->pose();
	m_target_pose.position = pose.position;
	m_target_pose.yaw = pose.yaw;
	m_target_pose.speed = 0;
	m_target_pose.yaw_rate = 0;
}

void PropulsionController::initMoveCommand(float speed, float accel, float deccel)
{
	// Compute speed progile
	m_speed_profile.update(m_trajectory_buffer.max_parameter(), speed, accel, deccel);

	// Compute direction by taking scalar product of current robot orientation vector with tangent of trajectory at origin
	auto target_point = m_trajectory_buffer.compute_point(0);
	float ux = cosf(m_target_pose.yaw);
	float uy = sinf(m_target_pose.yaw);
	m_direction = ux * target_point.tangent.x + uy * target_point.tangent.y > 0 ? Direction::Forward : Direction::Backward;

	// Set command end time
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));
}

void PropulsionController::initRotationCommand(float delta_yaw, float speed, float accel, float deccel)
{
	// Compute yaw ramp to go to target_angle from current target angle
	m_begin_yaw = m_target_pose.yaw;
	m_speed_profile.update(delta_yaw, speed, accel, deccel);
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));
}

bool PropulsionController::resetPose(float x, float y, float yaw)
{

	if(m_state == State::Inactive || m_state == State::Stopped)
	{
		RobotPose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.yaw = yaw;
		pose.speed = 0;
		pose.yaw_rate = 0;
		m_odometry->setPose(pose);
		m_target_pose.position = pose.position;
		m_target_pose.yaw = yaw;
		return true;
	} else
	{
		return false;
	}
}
bool PropulsionController::executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration)
{
	if(m_state != State::Stopped)
	{
		return false;
	}
	m_trajectory_buffer.push_segment(points, num_points);
	initMoveCommand(speed, acceleration, decceleration);
	m_state = State::FollowTrajectory;
	//m_translation_pid.set_config(m_config.translation_cruise_pid_config);
	m_low_level_controller.setConfig(m_config.low_level_config_cruise);
	m_low_level_controller.m_longi_control_level = 2;
	m_low_level_controller.m_yaw_control_level = 1;

	return true;
};

bool PropulsionController::executePointTo(Vector2D point, float speed, float acceleration, float decceleration)
{
	if(m_state != State::Stopped)
	{
		return false;
	}
	float diff_x = (point.x - m_current_pose.position.x);
	float diff_y = (point.y - m_current_pose.position.y);
	float target_yaw = atan2f(diff_y, diff_x);
	initRotationCommand(angleDiff(target_yaw, m_target_pose.yaw), speed, acceleration, decceleration);

	m_state = State::Rotate;
	return true;
};

bool PropulsionController::executeMoveTo(Vector2D point, float speed, float acceleration, float decceleration)
{
	Vector2D traj[2];
	traj[0] = m_target_pose.position;
	traj[1] = point;
	executeTrajectory(traj,2,speed, acceleration, decceleration);
	return true;
};

bool PropulsionController::executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel)
{
	if(m_state != State::Stopped)
	{
		return false;
	}
	initRotationCommand(delta_yaw, yaw_rate, accel, deccel);

	m_state = State::Rotate;
	return true;
}

bool PropulsionController::executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center)
{
	if(m_state != State::Stopped)
	{
		return false;
	}
	m_target_pose.speed = direction == Direction::Forward ? speed : -speed;
	m_direction = direction;
	m_reposition_border_normal = normal;
	m_reposition_border_distance = distance_to_center;
	m_reposition_hit = false;

	if(direction == Direction::Backward)
	{
		m_reposition_border_normal.x *= -1;
		m_reposition_border_normal.y *= -1;
		m_reposition_border_distance *= -1;
	}

	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_command_begin_time + 1500;

	m_state = State::Reposition;

	return true;
}

bool PropulsionController::executeTranslation(float distance, float speed, float accel, float deccel)
{
	Vector2D target;
	float ux = cos(m_target_pose.yaw);
	float uy = sin(m_target_pose.yaw);
	target.x = m_target_pose.position.x + ux * distance;
	target.y = m_target_pose.position.y + uy * distance;
	executeMoveTo(target, speed, accel, deccel);
}


messages::PropulsionTelemetry PropulsionController::getTelemetry() const
{
	messages::PropulsionTelemetry msg;
	msg.x = (int16_t)(m_current_pose.position.x * 4e3f);
	msg.y = (int16_t)(m_current_pose.position.y * 4e3f);
	msg.yaw = (int16_t)(m_current_pose.yaw * 32767 / M_PI);
	msg.speed = (int16_t)(m_current_pose.speed * 1000);
	msg.yaw_rate = (int16_t)(m_current_pose.yaw_rate * 1000);
	msg.acceleration = (int16_t)(m_current_pose.acceleration * 1000);
	msg.angular_acceleration = (int16_t)(m_current_pose.angular_acceleration * 1000);
	msg.left_encoder = m_odometry->leftEncoderValue();
	msg.right_encoder = m_odometry->rightEncoderValue();
	msg.left_pwm = (int16_t)(m_left_motor_pwm * 100);
	msg.right_pwm = (int16_t)(m_right_motor_pwm * 100);
	msg.state = (uint8_t)(state());
	msg.error = (uint8_t)(error());
	return msg;
}

messages::PropulsionTelemetryEx PropulsionController::getTelemetryEx() const
{
	messages::PropulsionTelemetryEx msg;
	msg.target_x = (int16_t)(m_target_pose.position.x * 4e3f);
	msg.target_y = (int16_t)(m_target_pose.position.y * 4e3f);
	msg.target_yaw = (int16_t)(m_target_pose.yaw * 32767 / M_PI);
	msg.target_speed = (int16_t)(m_target_pose.speed * 1000);
	msg.target_yaw_rate = (int16_t)(m_target_pose.yaw_rate * 1000);
	msg.longitudinal_error = (int16_t)(m_low_level_controller.m_longi_error * 4e3f);
	msg.lateral_error = (int16_t)(m_low_level_controller.m_lateral_error * 4e3f);
	msg.left_acc = m_odometry->m_left_accumulator;
	msg.right_acc = m_odometry->m_right_accumulator;
	return msg;
}

