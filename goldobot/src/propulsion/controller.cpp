#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
using namespace goldobot;

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

float angleDiff(float a, float b)
{
	float diff = a - b;
	if(diff > M_PI)
	{
		return diff - M_PI * 2;
	}
	if(diff < -M_PI)
	{
		return diff + M_PI * 2;
	}
	return diff;
}



TrapezoidalSpeedProfile::TrapezoidalSpeedProfile()
{
}

void TrapezoidalSpeedProfile::update(float distance, float speed, float accel, float deccel)
{
	float d_a = (speed * speed) * 0.5f / accel;
	float d_d = (speed * speed) * 0.5f / deccel;
	float d_c = distance - d_a - d_d;
	float t_a = 0;
	float t_d = 0;
	float t_c = 0;

	while(d_c < 0)
	{
	speed *= 0.95;
	d_a = (speed * speed) * 0.5f / accel;
	d_d = (speed * speed) * 0.5f / deccel;
	d_c = distance - d_a - d_d;
	}

	t_a = speed/accel;
	t_d = speed/deccel;
	t_c = distance/speed -t_a -t_d;

	m_t[0] = 0;
	m_t[1] = t_a;
	m_t[2] = t_a + t_c;
	m_t[3] = t_a + t_c + t_d;

	m_c0[0] = 0;
	m_c0[1] = d_a;
	m_c0[2] = d_a + d_c;
	m_c0[3] = distance;

	m_c1[0] = 0;
	m_c1[1] = speed;
	m_c1[2] = speed;
	m_c1[3] = 0;

	m_c2[0] = 0.5 * accel;
	m_c2[1] = 0;
	m_c2[2] = -0.5 * deccel;
	m_c2[3] = 0;


}

float TrapezoidalSpeedProfile::end_time()
{
	return m_t[3];
}

void TrapezoidalSpeedProfile::compute(float t, float* val, float* deriv, float* accel)
{
	int index = 0;

	while(index+1 < 4 && t < m_t[index+1])
	{
		index++;
	};

	float u = t - m_t[index];
	float c0 = m_c0[index];
	float c1 = m_c1[index];
	float c2 = m_c2[index];


	if(val != NULL)
	{
		*val = c0 + u*(c1 + u*c2);
	}
	if(deriv != NULL)
	{
		*deriv = c1 + u * 2*c2;
	}
	if(accel != NULL)
	{
		*accel = c2;
	}
}

PropulsionController::PropulsionController(SimpleOdometry* odometry):
		m_odometry(odometry),
		m_state(State::Inactive),
		m_left_motor_pwm(0),
		m_right_motor_pwm(0),
		m_time_base_ms(0),
		m_target_position({0,0}),
		m_target_yaw(0)
{
	m_state = State::Stopped;

	//test, 1/2 wheel spacing * speed feedforward
	m_yaw_pid.m_ffp = 0.1/1.7;
	m_translation_pid.m_kp = 2;

}

PropulsionController::State PropulsionController::state() const
{
	return m_state;
}

void PropulsionController::update()
{
	m_pose = m_odometry->pose();
	computePositionError();
	switch(m_state)
	{
	case State::Stopped:
		computeMotorsPwmStatic();
		break;
	case State::FollowTrajectory:
		{
			updateTargetPositions();
			computeMotorsPwmMoving();
			if(m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
			}
		}
		break;
	case State::Reposition:
		{
			updateTargetPositions();
			computeMotorsPwmMoving();
			// Check position error
			if(m_time_base_ms >= m_command_end_time)
			{
				m_state = State::Stopped;
			}
		}
		break;
	}
	// Update time base
	m_time_base_ms++;

	// Clamp outputs
	float pwm_limit = 0.3;
	if(m_left_motor_pwm > pwm_limit)
	{
		m_left_motor_pwm = pwm_limit;
	}
	if(m_left_motor_pwm < -pwm_limit)
	{
		m_left_motor_pwm = -pwm_limit;
	}
	if(m_right_motor_pwm > pwm_limit)
	{
		m_right_motor_pwm = pwm_limit;
	}
	if(m_right_motor_pwm < -pwm_limit)
	{
		m_right_motor_pwm = -pwm_limit;
	}

	// debug
	if(m_state ==State::FollowTrajectory || m_state == State::Reposition)
	{
		m_dbg_counter++;
		if(m_dbg_counter == 10)
		{
			m_dbg_counter = 0;
			m_dbg_pose_buffer[m_dbg_index] = m_pose;
			m_dbg_target_position_buffer[m_dbg_index] = m_target_position;
			m_dbg_target_yaw_buffer[m_dbg_index] = m_target_yaw;
			if(m_dbg_index < 500)
			{
				m_dbg_index++;
			}
		}
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

void PropulsionController::computeMotorsPwmStatic()
{
	// compute yaw command
	m_yaw_pid.set_target(m_target_yaw);
	float yaw_command = m_yaw_pid.update(m_pose.yaw, 1e-3f);

	// compute position command
	float translation_error = \
			(m_pose.position.x - m_target_position.x) * cosf(m_target_yaw) +\
			(m_pose.position.y - m_target_position.y) * sinf(m_target_yaw);
	m_translation_pid.set_target(0);
	float translation_command = m_translation_pid.update(translation_error, 1e-3f);
	m_left_motor_pwm = translation_command -yaw_command;
	m_right_motor_pwm =  translation_command + yaw_command;
}

void PropulsionController::computeMotorsPwmMoving()
{
	// Pure pursuit
	float ux = cosf(m_pose.yaw);
	float uy = sinf(m_pose.yaw);

	float diff_x = m_lookahead_position.x - m_pose.position.x;
	float diff_y = m_lookahead_position.y - m_pose.position.y;

	float rel_x = diff_x * ux + diff_y * uy;
	float rel_y = -diff_x * uy + diff_y * ux;

	float curvature = rel_y/(rel_x*rel_x + rel_y*rel_y);

	// Compute translation command
	// compute position command
	float translation_error = \
			(m_pose.position.x - m_target_position.x) * cosf(m_target_yaw) +\
			(m_pose.position.y - m_target_position.y) * sinf(m_target_yaw);
	m_translation_pid.set_target(0);
	float translation_command = m_translation_pid.update(translation_error, 1e-3f);

	translation_command += m_target_speed/1.7f;

	// compute yaw rate
	m_yaw_rate_pid.set_target(m_pose.speed * curvature);
	float yaw_rate_command = m_yaw_pid.update(m_pose.yaw_rate, 1e-3f);

	m_left_motor_pwm = translation_command -yaw_rate_command;
	m_right_motor_pwm =  translation_command + yaw_rate_command;
}

void PropulsionController::updateTargetPositions()
{
	float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;
	float parameter, accel;
	m_speed_profile.compute(t,&parameter,&m_target_speed,&accel);
	parameter = std::min(parameter, m_trajectory_buffer.max_parameter());
	float lookahead_distance = 0.2;
	float lookahead_parameter = parameter + lookahead_distance;



	auto target_point = m_trajectory_buffer.compute_point(parameter);
	m_target_position = target_point.position;
	m_target_yaw = atan2f(target_point.tangent.y, target_point.tangent.x);

	if(lookahead_parameter <= m_trajectory_buffer.max_parameter())
	{
		m_lookahead_position = m_trajectory_buffer.compute_point(lookahead_parameter).position;
	} else
	{
		auto end_point = m_trajectory_buffer.compute_point(m_trajectory_buffer.max_parameter());
		m_lookahead_position.x = end_point.position.x + lookahead_distance * end_point.tangent.x;
		m_lookahead_position.y = end_point.position.y + lookahead_distance * end_point.tangent.y;
	}
}

void PropulsionController::initMoveCommand(float speed, float accel, float deccel)
{
	m_speed_profile.update(m_trajectory_buffer.max_parameter(), speed, accel, deccel);
	m_command_begin_time = m_time_base_ms;
	m_command_end_time = m_time_base_ms + static_cast<uint32_t>(ceilf(1000 * m_speed_profile.end_time()));

}

void PropulsionController::computePositionError()
{

}

bool PropulsionController::executeTrajectory(Vector2D* points, int num_points, float speed, float acceleration, float decceleration)
{
	m_trajectory_buffer.push_segment(points, num_points);
	initMoveCommand(fabsf(speed), acceleration, decceleration);
	m_direction = speed >=0 ? Direction::Forward : Direction::Backward;

	m_dbg_index = 0;
	m_dbg_counter = 0;
	m_state = State::FollowTrajectory;
	return true;
};

bool PropulsionController::executeRepositioning(float speed, Vector2D normal, float distance_to_center)
{
	Vector2D points[2];
	points[0] = m_pose.position;
	points[1] = points[0];
	points[1].x += 0.5 * cosf(m_pose.yaw);
	points[1].y += 0.5 * sinf(m_pose.yaw);
	m_trajectory_buffer.push_segment(points, 2);
	initMoveCommand(fabsf(speed), 1,1);
	m_direction = speed >=0 ? Direction::Forward : Direction::Backward;

	m_dbg_index = 0;
	m_dbg_counter = 0;
	m_state = State::Reposition;
	return true;
}

