#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace goldobot;

PropulsionTask::PropulsionTask():
		m_controller(&m_odometry),
		m_message_queue(m_message_queue_buffer, 512)
{
}

const char* PropulsionTask::name() const
{
	return "propulsion";
}

void PropulsionTask::doStep()
{
	// Process messages
	while(m_message_queue.message_ready())
	{
		processMessage();
	}
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

void PropulsionTask::processMessage()
{
	auto message_type = (CommMessageType)m_message_queue.message_type();
	auto message_size = m_message_queue.message_size();
	auto& comm = Robot::instance().comm();

	switch(message_type)
	{
	case CommMessageType::DbgGetOdometryConfig:
		{
			auto config = m_odometry.config();
			comm.send_message(CommMessageType::DbgGetOdometryConfig, (char*)&config, sizeof(config));
			m_message_queue.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetOdometryConfig:
		{
			OdometryConfig config;
			m_message_queue.pop_message((unsigned char*)&config, sizeof(config));
			m_odometry.setConfig(config);
		}
		break;
	case CommMessageType::DbgGetPropulsionConfig:
		{
			auto config = m_controller.config();
			comm.send_message(CommMessageType::DbgGetPropulsionConfig, (char*)&config, sizeof(config));
			m_message_queue.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetPropulsionConfig:
		{
			PropulsionControllerConfig config;
			m_message_queue.pop_message((unsigned char*)&config, sizeof(config));
			m_controller.set_config(config);
		}
		break;
	case CommMessageType::CmdEmergencyStop:
		m_controller.emergency_stop();
		m_message_queue.pop_message(nullptr, 0);
		break;
	case CommMessageType::DbgSetPropulsionEnable:
		{
			uint8_t enabled;
			m_message_queue.pop_message((unsigned char*)&enabled, 1);
			if(enabled)
			{
				m_controller.enable();
			} else
			{
				m_controller.disable();
			}
		}
		break;
	case CommMessageType::DbgSetMotorsEnable:
		{
			uint8_t enabled;
			m_message_queue.pop_message((unsigned char*)&enabled, 1);
			Hal::set_motors_enable(enabled);
		}
		break;
	case CommMessageType::DbgSetMotorsPwm:
		{
			float pwm[2];
			m_message_queue.pop_message((unsigned char*)&pwm, 8);
			Hal::set_motors_pwm(pwm[0], pwm[1]);
		}
		break;
	case CommMessageType::DbgPropulsionExecuteTrajectory:
		onMsgExecuteTrajectory();
		break;
	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}
}

void PropulsionTask::onMsgExecuteTrajectory()
{
	auto& comm = Robot::instance().comm();

	unsigned char buff[14];
	m_message_queue.pop_message(buff,14);
	uint8_t pattern = buff[0];
	int8_t direction = buff[0];
	float speed = *(float*)(buff+2);
	float accel = *(float*)(buff+6);
	float deccel = *(float*)(buff+10);
	m_controller.reset_pose(0, 0, 0);
	uint8_t status = 0;
	comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);

	switch(pattern)
	{
	case 0:
		{
			Vector2D points[2] = {{0,0}, {0.5,0}};
			m_controller.executeTrajectory(points,2,speed, accel, deccel);
		}
		break;
	case 1:
		{
			Vector2D points[2] = {{0,0}, {-0.5,0}};
			m_controller.executeTrajectory(points,2,speed, accel, deccel);
		}
		break;
	case 2:
		{
			Vector2D points[3] = {{0,0}, {0.5,0}, {0.5,0.5}};
			m_controller.executeTrajectory(points,3,speed, accel, deccel);
		}
		break;
	case 3:
		{
			Vector2D points[3] = {{0,0}, {-0.5,0}, {-0.5,-0.5}};
			m_controller.executeTrajectory(points,3,speed, accel, deccel);
		}
		break;
	case 4:
		{
			Vector2D points[4] = {{0,0}, {0.5,0}, {0.5,0.5}, {1,0.5} };
			m_controller.executeTrajectory(points,4,speed, accel, deccel);
		}
		break;
	}
	while(m_controller.state() == PropulsionController::State::FollowTrajectory)
	{
		delay(1);
	}
	status = 1;
	comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);
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
	// Register for messages
	Robot::instance().mainExchange().subscribe({0,1000, &m_message_queue});
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
