#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

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

	// Test emergency stop
	if(Hal::get_gpio(2))
	{
		m_controller.emergency_stop();
	}
	m_controller.update();
	if(m_controller.state() != PropulsionController::State::Inactive)
	{
		Hal::set_motors_pwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());
	}

	// Send periodic telemetry messages
	m_telemetry_counter++;
	if(m_telemetry_counter == 50)
	{
		auto& comm = Robot::instance().comm();
		auto msg = m_controller.getTelemetryEx();
		comm.send_message(CommMessageType::PropulsionTelemetryEx,(const char*)&msg, sizeof(msg));
		m_telemetry_counter = 0;
	}
	if(m_telemetry_counter % 5 == 0)
	{
		auto& comm = Robot::instance().comm();
		auto msg = m_controller.getTelemetry();
		comm.send_message(CommMessageType::PropulsionTelemetry,(const char*)&msg, sizeof(msg));
	}
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
	// Set task to high
	set_priority(6);

	// Setup odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.reset(left, right);
	m_telemetry_counter = 0;

	while(1)
	{
		checkStateUpdate();
		if(m_state == Running)
		{
			doStep();
		}
		// Execute the propulsion control loop every system tick (1ms)
		delay_periodic(1);
	}
}
