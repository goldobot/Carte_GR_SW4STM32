#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"

using namespace goldobot;

PropulsionTask::PropulsionTask():
		m_controller(&m_odometry)
{
}

const char* PropulsionTask::name() const
{
	return "propulsion";
}


void PropulsionTask::doStep()
{
	// Update odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.update(left, right);
	m_controller.update();
	Hal::set_motors_pwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());

}

SimpleOdometry& PropulsionTask::odometry()
{
	return m_odometry;
}

PropulsionController& PropulsionTask::controller()
{
	return m_controller;
}


void PropulsionTask::taskFunction()
{
	// Setup odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.reset(left, right);

	while(1)
	{
		checkStateUpdate();
		if(m_state == Running)
		{
			doStep();
		}
		// Execute the propulsion control loop every system tick (1ms)
		delayTicks(1);
	}
}
