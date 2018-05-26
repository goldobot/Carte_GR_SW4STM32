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

struct PropulsionTelemetry
{
	int16_t x;//quarters of mm
	int16_t y;
	int16_t yaw;
	int16_t speed;// mm per second
	int16_t yaw_rate;// mradian per second
	int16_t acceleration;
	int16_t angular_acceleration;
	uint16_t left_encoder;
	uint16_t right_encoder;
	int8_t left_pwm;// percents
	int8_t right_pwm;
	uint8_t state;
	uint8_t error;
};

struct PropulsionTelemetryEx
{
	int16_t target_x;//quarters of mm
	int16_t target_y;
	int16_t target_yaw;
	int16_t target_speed;// mm per second
	int16_t target_yaw_rate;// mradian per second
	int16_t longitudinal_error;
	int16_t lateral_error;
	int16_t yaw_error;
	int16_t speed_error;
	int16_t yaw_rate_error;
};

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


	m_telemetry_counter++;

	if(m_telemetry_counter == 50)
	{
		auto& comm = Robot::instance().comm();
		PropulsionTelemetryEx msg;
		msg.target_x = (int16_t)(m_controller.m_target_position.x * 4e3f);
		msg.target_y = (int16_t)(m_controller.m_target_position.y * 4e3f);
		msg.target_yaw = (int16_t)(m_controller.m_target_yaw * 32767 / M_PI);
		msg.target_speed = (int16_t)(m_controller.m_target_speed * 1000);
		msg.target_yaw_rate = (int16_t)(m_controller.m_target_yaw_rate * 1000);
		msg.longitudinal_error = (int16_t)(m_controller.m_longitudinal_error * 4e3f);
		msg.lateral_error = (int16_t)(m_controller.m_lateral_error * 4e3f);
		comm.send_message(CommMessageType::PropulsionTelemetryEx,(const char*)&msg, sizeof(msg));
		m_telemetry_counter = 0;
	}
	if(m_telemetry_counter % 5 == 0)
	{
		auto& comm = Robot::instance().comm();
		PropulsionTelemetry msg;
		msg.x = (int16_t)(m_controller.m_pose.position.x * 4e3f);
		msg.y = (int16_t)(m_controller.m_pose.position.y * 4e3f);
		msg.yaw = (int16_t)(m_controller.m_pose.yaw * 32767 / M_PI);
		msg.speed = (int16_t)(m_controller.m_pose.speed * 1000);
		msg.yaw_rate = (int16_t)(m_controller.m_pose.yaw_rate * 1000);
		msg.acceleration = (int16_t)(m_controller.m_pose.acceleration * 1000);
		msg.angular_acceleration = (int16_t)(m_controller.m_pose.angular_acceleration * 1000);
		msg.left_encoder = left;
		msg.right_encoder = right;
		msg.left_pwm = (int16_t)(m_controller.m_left_motor_pwm * 100);
		msg.right_pwm = (int16_t)(m_controller.m_right_motor_pwm * 100);
		msg.state = (uint8_t)(m_controller.state());
		msg.error = (uint8_t)(m_controller.error());
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
