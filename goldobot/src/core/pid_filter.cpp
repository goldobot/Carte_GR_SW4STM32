#include <goldobot/core/pid_controller.hpp>

using namespace goldobot;

FeedForwardPIDController::FeedForwardPIDController()
{
	m_kp = 1;
	m_ffp = 0;
}


float FeedForwardPIDController::output() const
{
	return m_output;
}

void FeedForwardPIDController::set_target(float target, float target_derivative)
{
	m_current_target = target;
}

void FeedForwardPIDController::reset(float current_value)
{
	m_previous_value = current_value;
}

float FeedForwardPIDController::update(float current_value, float dt)
{
	return update(current_value, 0, dt);
}

float FeedForwardPIDController::update(float current_value, float current_derivative, float dt)
{
	m_previous_value = current_value;
	m_output = \
			m_ffp * m_current_target +\
			m_kp * (m_current_target - current_value);
	return m_output;
}

