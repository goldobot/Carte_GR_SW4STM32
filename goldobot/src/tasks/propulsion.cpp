#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace goldobot;

PropulsionTask::PropulsionTask():
		m_controller(&m_odometry),
		m_message_queue(m_message_queue_buffer, 512),
		m_telemetry_counter(0)
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

	// Check state change
	if(m_controller.state() != m_previous_state)
	{
		uint8_t buff[2];
		buff[0] = (uint8_t)m_controller.state();
		buff[1] = (uint8_t)m_previous_state;
		m_previous_state = m_controller.state();
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionStateChanged,(unsigned char*)buff,2);
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionStateChanged,(unsigned char*)buff,2);
	}
	if(m_controller.state() != PropulsionController::State::Inactive)
	{
		Hal::set_motors_pwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());
	}

	// Send periodic telemetry messages
	m_telemetry_counter++;
	if(m_telemetry_counter == 20)
	{
		auto msg = m_controller.getTelemetryEx();
		Robot::instance().mainExchangeOut().pushMessage(
				CommMessageType::PropulsionTelemetryEx,
				(unsigned char*)&msg, sizeof(msg));
		m_telemetry_counter = 0;

		//gpio debug
		uint32_t gpio = 0;
		for(int i=0; i<5; i++)
		{
			if(Hal::get_gpio(i)) gpio |= (1 << i);
		}
		Robot::instance().mainExchangeOut().pushMessage(
						CommMessageType::GPIODebug,
						(unsigned char*)&gpio, sizeof(gpio));
	}

	if(m_telemetry_counter % 5 == 0)
	{
		auto msg = m_controller.getTelemetry();
		Robot::instance().mainExchangeOut().pushMessage(
				CommMessageType::PropulsionTelemetry,
				(unsigned char*)&msg, sizeof(msg));
	}

}

void PropulsionTask::processMessage()
{
	auto message_type = (CommMessageType)m_message_queue.message_type();
	auto message_size = m_message_queue.message_size();

	switch(message_type)
	{
	case CommMessageType::DbgGetOdometryConfig:
		{
			auto config = m_odometry.config();
			Robot::instance().mainExchangeOut().pushMessage(
					CommMessageType::DbgGetOdometryConfig,
					(unsigned char*)&config, sizeof(config));
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
			Robot::instance().mainExchangeOut().pushMessage(
					CommMessageType::DbgGetPropulsionConfig,
					(unsigned char*)&config, sizeof(config));
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
	case CommMessageType::DbgPropulsionExecuteRotation:
		{
			float params[4];
			m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
			m_controller.executeRotation(params[0], params[1], params[2], params[3]);
		}
		break;
	case CommMessageType::DbgPropulsionExecutePointTo:
		{
			float params[5];
			m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
			m_controller.executePointTo(*(Vector2D*)(params), params[2], params[3], params[4]);
		}
		break;
	case CommMessageType::DbgPropulsionExecuteMoveTo:
			{
				float params[5];
				m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
				m_controller.executeMoveTo(*(Vector2D*)(params), params[2], params[3], params[4]);
			}
			break;
	case CommMessageType::DbgPropulsionSetPose:
		{
			float pose[3];
			m_message_queue.pop_message((unsigned char*)&pose, 12);
			m_controller.reset_pose(pose[0], pose[1], pose[2]);
		}
		break;
	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}
}

void PropulsionTask::onMsgExecuteTrajectory()
{
	unsigned char buff[76];//12 for traj params and 8*8 for points
	auto msg_size = m_message_queue.message_size();
	m_message_queue.pop_message(buff,76);
	float speed = *(float*)(buff);
	float accel = *(float*)(buff+4);
	float deccel = *(float*)(buff+8);
	Vector2D* points = (Vector2D*)(buff+12);
	int num_points = (msg_size-12)/sizeof(Vector2D);
	m_controller.executeTrajectory(points,num_points,speed, accel, deccel);
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
	Robot::instance().mainExchangeIn().subscribe({0,1000, &m_message_queue});
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
