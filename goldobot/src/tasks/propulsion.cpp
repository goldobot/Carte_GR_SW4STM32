#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

using namespace goldobot;

PropulsionTask::PropulsionTask():
		m_controller(&m_odometry)
{
}

const char* PropulsionTask::name() const
{
	return "propulsion";
}

struct PropulsionTelemetry
{
	int16_t x;//quarters of mm
	int16_t y;
	int16_t yaw;
	int16_t speed;// mm per second
	int16_t yaw_rate;// mradian per second
	uint16_t left_encoder;
	uint16_t right_encoder;
	int8_t left_pwm;// percents
	int8_t right_pwm;
};

void PropulsionTask::doStep()
{
	// Update odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.update(left, right);
	m_controller.update();
	Hal::set_motors_pwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());

	m_telemetry_counter++;

	if(m_telemetry_counter == 10)
	{
		auto& comm = Robot::instance().comm();
		PropulsionTelemetry msg;
		msg.x = (int16_t)(m_controller.m_pose.position.x * 4e3f);
		msg.y = (int16_t)(m_controller.m_pose.position.y * 4e3f);
		comm.send_message((uint16_t)CommMessageType::PropulsionTelemetry,(const char*)&msg, sizeof(msg));
		m_telemetry_counter = 0;
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
	// Set task to highest priority

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
		delayTicks(1);
	}
}
