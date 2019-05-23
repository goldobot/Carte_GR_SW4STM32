#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
using namespace goldobot;

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

float clampAngle(float a)
{
	while(a > M_PI)
	{
		a = a - M_PI * 2;
	}
	while (a < -M_PI)
	{
		a = a + M_PI * 2;
	}
	return a;
}

static float clamp(float val, float min_val, float max_val)
{
	if(val < min_val)
	{
		return min_val;
	} else if(val > max_val)
	{
		return max_val;
	} else
	{
		return val;
	}
}

float angleDiff(float a, float b)
{
	float diff = a - b;
	return clampAngle(diff);
}



PropulsionController::PropulsionController(SimpleOdometry* odometry):
		m_odometry(odometry),
		m_state(State::Inactive),
		m_error(Error::None),
		m_left_motor_pwm(0),
		m_right_motor_pwm(0),
		m_time_base_ms(0),
		m_target_position({0,0}),
		m_target_yaw(0),
		m_target_speed(0),
		m_target_yaw_rate(0),
		m_lookahead_position({0,0}),
		m_longitudinal_error(0),
		m_lateral_error(0),
		m_yaw_error(0),
		m_speed_error(0),
		m_yaw_rate_error(0),
		m_pwm_limit(1),
		m_control_translation(false),
		m_control_yaw(false),
		m_control_speed(false),
		m_control_yaw_rate(false)
{
	test = false;
	m_mutex = xSemaphoreCreateMutex();
}

void PropulsionController::enable()
{
	if(m_state != State::Inactive)
	{
		return;
	}
	m_pose = m_odometry->pose();
	m_target_position = m_pose.position;
	m_target_yaw = m_pose.yaw;
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
void PropulsionController::set_config(const PropulsionControllerConfig& config)
{
	m_config = config;
	m_speed_pid.set_config(config.speed_pid_config);
	m_yaw_rate_pid.set_config(config.yaw_rate_pid_config);
	m_translation_pid.set_config(config.translation_pid_config);
	m_yaw_pid.set_config(config.yaw_pid_config);
}

void PropulsionController::clear_error()
{
	m_state = State::Stopped;
	m_error = Error::None;
}

void PropulsionController::emergency_stop()
{
	if(!test)
	{
		return;
	}
	if(m_state == State::FollowTrajectory || m_state == State::PointTo)
	{
		m_state = State::EmergencyStop;
	}
}

void PropulsionController::update()
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
	{
	}
	m_pose = m_odometry->pose();
	switch(m_state)
	{
	case State::Inactive:
		break;
	case State::Stopped:
		m_pwm_limit = m_config.static_pwm_limit;
		updateMotorsPwm();
		if(fabsf(m_longitudinal_error) > 0.1f)
		{
			m_state = State::Error;
		}
		break;
	case State::FollowTrajectory:
		{
			m_pwm_limit = m_config.moving_pwm_limit;
			updateTargetPositions();
			updateMotorsPwm();
			if(m_time_base_ms >= m_command_end_time)
			{
				on_trajectory_exit();
				on_stopped_enter();
			}
		}
		break;
	case State::PointTo:
		{
			m_pwm_limit = m_config.moving_pwm_limit;
			updateTargetYaw();
			updateMotorsPwm();
			if (m_time_base_ms >= m_command_end_time)
			{
				on_rotation_exit();
				on_stopped_enter();
			}
		}
		break;
	case State::Reposition:
		{
			m_pwm_limit = m_config.repositioning_pwm_limit;
			updateReposition();
			updateMotorsPwm();
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
			if (fabsf(m_pose.speed) < 0.01 && fabsf(m_pose.yaw_rate) < 0.1)
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
	case State::Test:
		{
			m_pwm_limit = m_config.moving_pwm_limit;
			update_test();
			updateMotorsPwm();

			if (m_time_base_ms >= m_command_end_time)
			{
				on_test_exit();
			}
		}
		break;
	}
	// Update time base
	m_time_base_ms++;

	// Clamp outputs
	m_left_motor_pwm = clamp(m_left_motor_pwm, -m_pwm_limit, m_pwm_limit);
	m_right_motor_pwm = clamp(m_right_motor_pwm, -m_pwm_limit, m_pwm_limit);
	xSemaphoreGive(m_mutex);
}


float PropulsionController::leftMotorPwm()
{
	return m_left_motor_pwm;
}

float PropulsionController::PropulsionController::rightMotorPwm()
{
	return m_right_motor_pwm;
}

RobotPose PropulsionController::target_pose() const
{
	RobotPose pose;
	pose.position = m_target_position;
	pose.yaw = m_target_yaw;
	return pose;

}

void PropulsionController::updateMotorsPwm()
{
	// Current robot frame direction
	float ux = cosf(m_pose.yaw);
	float uy = sinf(m_pose.yaw);
	
	// Compute position error
	float diff_x = (m_pose.position.x - m_target_position.x);
	float diff_y = (m_pose.position.y - m_target_position.y);

	m_longitudinal_error = diff_x * ux + diff_y * uy;
	m_lateral_error = -diff_x * uy + diff_y * ux;
	m_yaw_error = angleDiff(m_pose.yaw, m_target_yaw);
	m_speed_error = m_pose.speed - m_target_speed;
	
	// Compute translation and speed command
	// Two nested PID controllers are used
	// First is computing speed correction based on position error
	// Second is computing motors pwm based on speed

	float translation_command = 0;
	float speed_command = 0;

	if(m_control_translation)
	{
		m_translation_pid.set_target(0, 0);
		translation_command = m_translation_pid.update(m_longitudinal_error);
	}

	if(m_control_speed)
	{
		m_speed_pid.set_target(m_target_speed + translation_command);
		speed_command = m_speed_pid.update(m_pose.speed);
	}
	else
	{
		speed_command = m_target_speed + translation_command;
	}

	// Compute yaw and yaw_rate command
	// Two nested PID controllers are used
	// First is computing speed correction based on yaw error
	// Second is computing motors pwm difference based on yaw rate

	float yaw_command = 0;
	float yaw_rate_command = 0;
	if (m_control_yaw)
	{
		m_yaw_pid.set_target(0);
		yaw_command = m_yaw_pid.update(m_yaw_error);
	}

	if(m_control_yaw_rate)
	{
		m_yaw_rate_pid.set_target(m_target_yaw_rate + yaw_command);
		yaw_rate_command = m_yaw_rate_pid.update(m_pose.yaw_rate);
	}
	else
	{
		yaw_rate_command = m_target_yaw_rate + yaw_command;
	}

	m_left_motor_pwm = speed_command - yaw_rate_command;
	m_right_motor_pwm =  speed_command + yaw_rate_command;
}


void PropulsionController::update_test()
{
	// Current robot frame direction

	int step_time = 500;
	switch(m_test_pattern)
	{
	case TestPattern::SpeedSteps:
		step_time = 500;
		break;
	case TestPattern::YawSteps:
		step_time = 1000;
		break;
	case TestPattern::PositionStaticSteps:
			step_time = 1000;
			break;

	}

	int step_index = (m_time_base_ms - m_command_begin_time)/step_time;
	if(step_index < 0)
	{
		step_index = 0;
	}
	if(step_index > 8)
	{
		step_index = 8;
	}

	switch(m_test_pattern)
	{
	case TestPattern::SpeedSteps:
	{
		float speed_steps[] = {
				0.0,
				0.1,
				0.3,
				0.6,
				0,
				-0.1,
				-0.3,
				-0.6,
				0
				};

				m_target_speed = speed_steps[step_index];
	}
	break;
	case TestPattern::YawRateSteps:
	{
		float speed_steps[] = {
				0.0,
				0.2,
				0.6,
				2,
				0,
				-0.2,
				-0.6,
				-2,
				0
				};

				m_target_yaw_rate = speed_steps[step_index];
	}
	break;
	case TestPattern::YawSteps:
		{
			float speed_steps[] = {
					0.0,
					0.1,
					0.3,
					0.6,
					0,
					-0.1,
					-0.3,
					-0.6,
					0
					};

					m_target_yaw = speed_steps[step_index];
					m_target_yaw_rate = 0;
		}
		break;
	case TestPattern::PositionStaticSteps:
			{
				float speed_steps[] = {
						0.0,
						0.05,
						0.15,
						0.30,
						0.30,
						0.50,
						0.50,
						0.25,
						0
						};
					float ux = cosf(m_test_initial_yaw);
					float uy = sinf(m_test_initial_yaw);
					m_target_position.x = m_test_initial_position.x+ux*speed_steps[step_index];
					m_target_position.y = m_test_initial_position.y+uy*speed_steps[step_index];
					m_target_speed = 0;
			}
	default:
		break;
	}
}

void PropulsionController::updateTargetPositions()
{
	// Compute current distance on trajectory target
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, speed, accel;
	m_speed_profile.compute(t,&parameter,&speed,&accel);
	parameter = std::min(parameter, m_trajectory_buffer.max_parameter());

	// Compute position of lookahead point in front of current position
	float lookahead_distance = m_config.lookahead_distance + fabsf(m_target_speed) * m_config.lookahead_time;
	float lookahead_parameter = parameter + lookahead_distance;

	// Compute target position
	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_position = target_point.position;


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
	float ux = cosf(m_pose.yaw);
	float uy = sinf(m_pose.yaw);

	m_target_speed = speed * (ux * target_point.tangent.x + uy * target_point.tangent.y);

	// Pure pursuit computation, update target yaw rate
	float diff_x = m_lookahead_position.x - m_pose.position.x;
	float diff_y = m_lookahead_position.y - m_pose.position.y;

	float rel_x = diff_x * ux + diff_y * uy;
	float rel_y = -diff_x * uy + diff_y * ux;

	float curvature = 2.0 * rel_y / (rel_x*rel_x + rel_y*rel_y);
	m_target_yaw_rate = m_pose.speed * curvature;
	m_target_yaw = m_pose.yaw;
}

void PropulsionController::updateTargetYaw()
{
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, accel;
	m_speed_profile.compute(t, &parameter, &m_target_yaw_rate, &accel);
	m_target_yaw = clampAngle(m_begin_yaw + parameter);
}

void PropulsionController::updateReposition()
{
	float ux = cosf(m_target_yaw);
	float uy = sinf(m_target_yaw);

	m_target_position.x += ux * m_target_speed * 1e-3;
	m_target_position.y += uy * m_target_speed * 1e-3;

	if(fabs(m_longitudinal_error) > 0.06 && ! m_reposition_hit)
	{
		m_reposition_hit = true;
		m_command_end_time = m_time_base_ms + 500;
	}
};

void PropulsionController::on_stopped_enter()
{
	m_state = State::Stopped;
	m_control_translation = true;
	m_control_speed = true;
	m_control_yaw = true;
	m_control_yaw_rate = true;

	m_target_speed = 0;
	m_target_yaw_rate = 0;

	m_translation_pid.set_config(m_config.translation_pid_config);
	m_pwm_limit = m_config.static_pwm_limit;
}

void PropulsionController::on_trajectory_exit()
{
	float parameter = m_trajectory_buffer.max_parameter();
	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_position = target_point.position;
	m_target_yaw = atan2f(target_point.tangent.y, target_point.tangent.x);

	if(m_direction == Direction::Backward)
	{
		m_target_yaw = clampAngle(m_target_yaw + M_PI);
	}
}

void PropulsionController::on_rotation_exit()
{
	// \todo cleanup
	float t = (m_command_end_time - m_command_begin_time) * 1e-3f;
	float parameter, accel;
	m_speed_profile.compute(t, &parameter, &m_target_yaw_rate, &accel);
	m_target_yaw = clampAngle(m_begin_yaw + parameter);
}

void PropulsionController::on_reposition_exit()
{
	if(m_reposition_hit)
	{
		repositionReconfigureOdometry();
		// Reset integral terms of pids
		m_translation_pid.reset();
		m_speed_pid.reset();
		m_yaw_pid.reset();
		m_yaw_rate_pid.reset();
	}
}

void PropulsionController::on_test_exit()
{
	// Reset PIDs
	m_translation_pid.reset();
	m_speed_pid.reset();
	m_yaw_pid.reset();
	m_yaw_rate_pid.reset();

	m_target_position = m_pose.position;
	m_target_yaw = m_pose.yaw;
	on_stopped_enter();
}

void PropulsionController::repositionReconfigureOdometry()
{
	m_odometry->measureLineNormal(m_reposition_border_normal, m_reposition_border_distance);
	auto pose = m_odometry->pose();
	m_target_position = pose.position;
	m_target_yaw = pose.yaw;
	m_target_speed = 0;
	m_target_yaw_rate = 0;
}

void PropulsionController::initMoveCommand(float speed, float accel, float deccel)
{
	// Compute speed progile
	m_speed_profile.update(m_trajectory_buffer.max_parameter(), speed, accel, deccel);

	// Compute direction by taking scalar product of current robot orientation vector with tangent of trajectory at origin
	auto target_point = m_trajectory_buffer.compute_point(0);
	float ux = cosf(m_target_yaw);
	float uy = sinf(m_target_yaw);
	m_direction = ux * target_point.tangent.x + uy * target_point.tangent.y > 0 ? Direction::Forward : Direction::Backward;

	// Set command end time
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));
}

void PropulsionController::initRotationCommand(float delta_yaw, float speed, float accel, float deccel)
{
	// Compute yaw ramp to go to target_angle from current target angle
	m_begin_yaw = m_target_yaw;
	m_speed_profile.update(delta_yaw, speed, accel, deccel);
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));
}

bool PropulsionController::reset_pose(float x, float y, float yaw)
{
	//while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
	//	{
	//	}
	if(m_state == State::Inactive || m_state == State::Stopped)
	{
		RobotPose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.yaw = yaw;
		pose.speed = 0;
		pose.yaw_rate = 0;
		m_odometry->setPose(pose);
		m_target_position = pose.position;
		m_target_yaw = yaw;
		//xSemaphoreGive(m_mutex);
	//	return true;
	} else
	{
	//	xSemaphoreGive(m_mutex);
	//	return false;
	}
}
bool PropulsionController::executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration)
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
		{
		}
	if(m_state != State::Stopped)
	{
		xSemaphoreGive(m_mutex);
		return false;
	}
	m_trajectory_buffer.push_segment(points, num_points);
	initMoveCommand(speed, acceleration, decceleration);
	m_state = State::FollowTrajectory;
	m_translation_pid.set_config(m_config.translation_cruise_pid_config);

	m_control_translation = true;
	m_control_speed = true;
	m_control_yaw = true /* false */; /* FIXME : DEBUG */
	m_control_yaw_rate = true;

	xSemaphoreGive(m_mutex);
	return true;
};

bool PropulsionController::executePointTo(Vector2D point, float speed, float acceleration, float decceleration)
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
		{
		}
	if(m_state != State::Stopped)
	{
		xSemaphoreGive(m_mutex);
		return false;
	}
	float diff_x = (point.x - m_pose.position.x);
	float diff_y = (point.y - m_pose.position.y);
	float target_yaw = atan2f(diff_y, diff_x);
	initRotationCommand(angleDiff(target_yaw, m_target_yaw), speed, acceleration, decceleration);

	m_state = State::PointTo;
	xSemaphoreGive(m_mutex);
	return true;
};

bool PropulsionController::executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel)
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
		{
		}
	if(m_state != State::Stopped)
	{
		xSemaphoreGive(m_mutex);
		return false;
	}
	initRotationCommand(delta_yaw, yaw_rate, accel, deccel);

	m_state = State::PointTo;
	xSemaphoreGive(m_mutex);
	return true;
}

bool PropulsionController::executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center)
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
		{
		}
	if(m_state != State::Stopped)
	{
		xSemaphoreGive(m_mutex);
		return false;
	}
	m_target_speed = direction == Direction::Forward ? speed : -speed;
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

	xSemaphoreGive(m_mutex);
	return true;
}

void PropulsionController::executeTest(TestPattern pattern)
{
	while(xSemaphoreTake(m_mutex, 1) != pdTRUE)
		{
		}
	if(m_state != State::Stopped)
		{
		xSemaphoreGive(m_mutex);
			return;
		}
	m_test_pattern = pattern;
	m_test_initial_position = m_target_position;
	m_test_initial_yaw = m_target_yaw;

	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_command_begin_time + 4000;
	m_state = State::Test;

	switch(m_test_pattern)
	{
	case TestPattern::SpeedSteps:
		m_control_translation = false;
		m_control_speed = true;
		m_control_yaw = false;
		m_control_yaw_rate = false;
		m_command_end_time = m_command_begin_time + 4500;
		break;
	case TestPattern::PositionStaticSteps:
		m_control_translation = true;
		m_control_speed = true;
		m_control_yaw = false;
		m_control_yaw_rate = false;
		m_command_end_time = m_command_begin_time + 9000;
		break;
	case TestPattern::YawRateSteps:
		m_control_translation = false;
		m_control_speed = false;
		m_control_yaw = false;
		m_control_yaw_rate = true;
		m_command_end_time = m_command_begin_time + 4500;
		break;
	case TestPattern::YawSteps:
		m_control_translation = false;
		m_control_speed = false;
		m_control_yaw = true;
		m_control_yaw_rate = true;
		m_command_end_time = m_command_begin_time + 9000;
		break;
	default:
		break;
	}
	xSemaphoreGive(m_mutex);
}

