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
	if (a > M_PI)
	{
		return a - M_PI * 2;
	}
	if (a < -M_PI)
	{
		return a + M_PI * 2;
	}
	return a;
}

float angleDiff(float a, float b)
{
	float diff = a - b;
	return clampAngle(diff);
}



PropulsionController::PropulsionController(SimpleOdometry* odometry):
		m_odometry(odometry),
		m_state(State::Inactive),
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
		m_pwm_limit(1)
{
	m_state = State::Stopped;

	PIDConfig speed_pid_config;
	PIDConfig yaw_rate_pid_config;
	PIDConfig translation_pid_config; 
	PIDConfig yaw_pid_config;

	speed_pid_config.period = 1e-3f;
	yaw_rate_pid_config.period = 1e-3f;
	translation_pid_config.period = 1e-3f;
	yaw_pid_config.period = 1e-3f;

	translation_pid_config.kp = 5;
	yaw_pid_config.kp = 5;


	// Configure speed pid
	speed_pid_config.feed_forward = 0.64f;
	speed_pid_config.kp = 0.5f;
	speed_pid_config.lim_iterm = 0.2;

	// Configure yaw rate pid
	yaw_rate_pid_config.feed_forward = 0.1f / 1.7f;
	yaw_rate_pid_config.kp = 0.2;
	//yaw_rate_pid_config.kd = 1;

	m_speed_pid.set_config(speed_pid_config);
	m_yaw_rate_pid.set_config(yaw_rate_pid_config);
	m_translation_pid.set_config(translation_pid_config);
	m_yaw_pid.set_config(yaw_pid_config);

	//test, 1/2 wheel spacing * speed feedforward
	//m_yaw_rate_pid.m_ffp = 0.1/1.7;
	//m_yaw_rate_pid.m_kp = 0;

	//m_translation_pid.m_kp = 5;
	//m_translation_pid.m_ffd = 1/1.7;
	//m_translation_pid.m_kd = 1;

}

PropulsionController::State PropulsionController::state() const
{
	return m_state;
}

void PropulsionController::set_speed_feedforward(float ff)
{
	m_speed_pid.set_feedforward(ff);

}
void PropulsionController::set_speed_kp(float kp)
{
	m_speed_pid.set_kp(kp);
}

void PropulsionController::set_speed_kd(float kd)
{
	m_speed_pid.set_kd(kd);
}

void PropulsionController::set_speed_ki(float ki)
{
	m_speed_pid.set_ki(ki);
}

void PropulsionController::set_translation_kp(float kp)
{
	m_translation_pid.set_kp(kp);
}

void PropulsionController::set_translation_kd(float kd)
{
	m_translation_pid.set_kd(kd);
}

void PropulsionController::set_translation_ki(float ki)
{
	m_translation_pid.set_ki(ki);
}


void PropulsionController::update()
{
	m_pose = m_odometry->pose();
	switch(m_state)
	{
	case State::Stopped:
		updateMotorsPwm();
		break;
	case State::FollowTrajectory:
		{
			updateTargetPositions();
			updateMotorsPwm();
			if(m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
			}
		}
		break;
	case State::PointTo:
		{
			updateTargetYaw();
			updateMotorsPwm();
			if (m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
			}
		}
		break;
	case State::Reposition:
		{
			updateReposition();
			updateMotorsPwm();
			// Check position error
			if(m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
				if(m_reposition_hit)
				{
					repositionReconfigureOdometry();
				}
			}
		}
		break;
	case State::Test:
		{
			updateMotorsPwmTest();
			if (m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
			}
		}
		break;
	}
	// Update time base
	m_time_base_ms++;

	// Clamp outputs
	if(m_left_motor_pwm > m_pwm_limit)
	{
		m_left_motor_pwm = m_pwm_limit;
	}
	if(m_left_motor_pwm < -m_pwm_limit)
	{
		m_left_motor_pwm = -m_pwm_limit;
	}
	if(m_right_motor_pwm > m_pwm_limit)
	{
		m_right_motor_pwm = m_pwm_limit;
	}
	if(m_right_motor_pwm < -m_pwm_limit)
	{
		m_right_motor_pwm = -m_pwm_limit;
	}
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

	if (m_state == State::FollowTrajectory)
	{
		// Pure pursuit computation, update target yaw rate
		float diff_x = m_lookahead_position.x - m_pose.position.x;
		float diff_y = m_lookahead_position.y - m_pose.position.y;

		float rel_x = diff_x * ux + diff_y * uy;
		float rel_y = -diff_x * uy + diff_y * ux;

		float curvature = 2 * rel_y / (rel_x*rel_x + rel_y * rel_y);
		m_target_yaw_rate = m_pose.speed * curvature;
	}
	
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
	m_translation_pid.set_target(0, 0);
	float translation_command = m_translation_pid.update(m_longitudinal_error);

	m_speed_pid.set_target(m_target_speed + translation_command);
	float speed_command = m_speed_pid.update(m_pose.speed);

	// Compute yaw and yaw_rate command
	// Two nested PID controllers are used
	// First is computing speed correction based on yaw error
	// Second is computing motors pwm difference based on yaw rate
	float yaw_command;
	if (m_state == State::FollowTrajectory)
	{
		// In trajectory following mode, yaw rate is controlled instead of yaw
		// So the yaw PID is bypassed
		m_yaw_error = 0;
		yaw_command = 0;
	}
	else
	{
		m_yaw_pid.set_target(0);
		yaw_command = m_yaw_pid.update(m_yaw_error);
	}

	m_yaw_rate_pid.set_target(m_target_yaw_rate + yaw_command);
	float yaw_rate_command = m_yaw_rate_pid.update(m_pose.yaw_rate);

	m_left_motor_pwm = speed_command -yaw_rate_command;
	m_right_motor_pwm =  speed_command + yaw_rate_command;
}


void PropulsionController::updateMotorsPwmTest()
{
	// Current robot frame direction
	float ux = cosf(m_pose.yaw);
	float uy = sinf(m_pose.yaw);

	if(m_test_pattern == TestPattern::SpeedSteps)
	{
		float speed_steps[] = {
				0,
				0.1,
				0.3,
				0.3,
				0.6,
				0,
				-0.2,
				0
		};
		int step_index = (m_time_base_ms - m_command_begin_time)/500;
		if(step_index < 0)
		{
			step_index = 0;
		}
		if(step_index > 7)
		{
			step_index = 7;
		}
		m_target_speed = speed_steps[step_index];

		m_speed_pid.set_target(m_target_speed);
		float speed_command = m_speed_pid.update(m_pose.speed);

		m_left_motor_pwm = speed_command;
		m_right_motor_pwm = speed_command;

		return;

	}
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
	m_translation_pid.set_target(0, 0);
	float translation_command = m_translation_pid.update(m_longitudinal_error);

	m_speed_pid.set_target(m_target_speed + translation_command);
	float speed_command = m_speed_pid.update(m_pose.speed);

	// Compute yaw and yaw_rate command
	// Two nested PID controllers are used
	// First is computing speed correction based on yaw error
	// Second is computing motors pwm difference based on yaw rate
	float yaw_command;
	if (m_state == State::FollowTrajectory)
	{
		// In trajectory following mode, yaw rate is controlled instead of yaw
		// So the yaw PID is bypassed
		m_yaw_error = 0;
		yaw_command = 0;
	}
	else
	{
		m_yaw_pid.set_target(0);
		yaw_command = m_yaw_pid.update(m_yaw_error);
	}

	m_yaw_rate_pid.set_target(m_target_yaw_rate + yaw_command);
	float yaw_rate_command = m_yaw_rate_pid.update(m_pose.yaw_rate);

	m_left_motor_pwm = speed_command - yaw_rate_command;
	m_right_motor_pwm = speed_command + yaw_rate_command;

	if(m_state == State::Reposition && m_reposition_hit)
	{
		float reposition_pwm = m_direction == Direction::Forward ? 0.5 : -0.5;
		m_left_motor_pwm = reposition_pwm;
		m_right_motor_pwm = reposition_pwm;
	}
}

void PropulsionController::updateTargetPositions()
{
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, speed, accel;
	m_speed_profile.compute(t,&parameter,&speed,&accel);
	parameter = std::min(parameter, m_trajectory_buffer.max_parameter());

	float lookahead_distance = 0.15;
	float lookahead_parameter = parameter + lookahead_distance;

	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_position = target_point.position;
	m_target_yaw = atan2f(target_point.tangent.y, target_point.tangent.x);

	// Robot target yaw is reversed compared to trajectory tangent if moving backward
	if(m_direction == Direction::Backward)
	{
		m_target_yaw *= -1;
	}

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

	if(fabs(m_longitudinal_error) > 0.04 && ! m_reposition_hit)
	{
		m_reposition_hit = true;
		m_command_end_time = m_time_base_ms + 200;
	}
};

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
		return true;
	} else
	{
		return false;
	}
}
bool PropulsionController::executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration)
{
	m_trajectory_buffer.push_segment(points, num_points);
	initMoveCommand(speed, acceleration, decceleration);
	m_state = State::FollowTrajectory;
	return true;
};

bool PropulsionController::executePointTo(Vector2D point, float speed, float acceleration, float decceleration)
{
	float diff_x = (point.x - m_pose.position.x);
	float diff_y = (point.y - m_pose.position.y);
	float target_yaw = atan2f(diff_y, diff_x);
	initRotationCommand(angleDiff(target_yaw, m_target_yaw), speed, acceleration, decceleration);

	m_state = State::PointTo;
	return true;
};

bool PropulsionController::executeRotation(float delta_yaw, float yaw_rate, float accel, float deccel)
{
	initRotationCommand(delta_yaw, yaw_rate, accel, deccel);

	m_state = State::PointTo;
	return true;
}

bool PropulsionController::executeRepositioning(Direction direction, float speed, Vector2D normal, float distance_to_center)
{
	m_target_speed = direction == Direction::Forward ? speed : -speed;
	m_direction = direction;
	m_pwm_limit = 0.25;
	m_reposition_hit = false;

	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_command_begin_time + 1000;

	m_state = State::Reposition;
	return true;
}

void PropulsionController::executeTest(TestPattern pattern)
{
	m_test_pattern = pattern;
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_command_begin_time + 4000;
	m_state = State::Test;
}

