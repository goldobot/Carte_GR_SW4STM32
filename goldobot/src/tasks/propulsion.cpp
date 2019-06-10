#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace goldobot;

PropulsionTask::PropulsionTask():
		m_controller(&m_odometry),
		m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)),
		m_urgent_message_queue(m_urgent_message_queue_buffer, sizeof(m_urgent_message_queue_buffer))

{
}

const char* PropulsionTask::name() const
{
	return "propulsion";
}

void PropulsionTask::doStep()
{
	// Process urgent messages
	while(m_urgent_message_queue.message_ready())
	{
		processUrgentMessage();
	}

	// Process messages
	while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::Stopped)
	{
		processMessage();
	}

	while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::ManualControl)
		{
			processMessage();
		}

	// adversary detection
	if(Hal::get_gpio(2) && m_controller.state() == PropulsionController::State::FollowTrajectory&& m_adversary_detection_enabled)
	{
		m_controller.emergencyStop();
	}
	// Goldenium hack
	if(m_recalage_goldenium_armed)
	{
		if(Robot::instance().side() == Side::Yellow && Robot::instance().sensorsState() & (1 << 42))
		{
			auto pose = m_odometry.pose();
			pose.position.y = 1;
			m_odometry.setPose(pose);
			m_recalage_goldenium_armed = false;

		}

		if(Robot::instance().side() == Side::Purple && Robot::instance().sensorsState() & (1 << 42))
		{
			auto pose = m_odometry.pose();
			//pose.position.y = ;
			m_odometry.setPose(pose);
			m_recalage_goldenium_armed = false;
		}

	}
	// Update odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.update(left, right);
	m_controller.update();

	while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::Stopped)
	{
		processMessage();
	}

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
	}

	if(m_telemetry_counter == 40)
	{
		float msg[3];
		msg[0] = m_odometry.pose().position.x;
		msg[1] = m_odometry.pose().position.y;
		msg[2] = m_odometry.pose().yaw;

		Robot::instance().mainExchangeOut().pushMessage(
				CommMessageType::PropulsionPose,
				(unsigned char*)&msg, sizeof(msg));
		m_telemetry_counter = 0;
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

	switch(message_type)
	{
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
	case CommMessageType::PropulsionExecuteTranslation:
			{
				float params[4];
				m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
				m_controller.executeTranslation(params[0], params[1], params[2], params[3]);
			}
			break;
	case CommMessageType::DbgPropulsionExecutePointTo:
		{
			float params[5];
			m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
			m_controller.executePointTo(*(Vector2D*)(params), params[2], params[3], params[4]);
		}
		break;
	case CommMessageType::PropulsionExecuteFaceDirection:
		{
			float params[4];
			m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
			m_controller.executeFaceDirection(params[0], params[1], params[2], params[3]);
		}
		break;
	case CommMessageType::DbgPropulsionExecuteMoveTo:
			{
				float params[5];
				m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
				m_controller.executeMoveTo(*(Vector2D*)(params), params[2], params[3], params[4]);
			}
			break;
	case CommMessageType::DbgPropulsionExecuteReposition:
			{
				float params[2];
				m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
				m_controller.executeRepositioning(params[0], params[1]);
			}
			break;
	case CommMessageType::PropulsionEnterManualControl:
		m_controller.enterManualControl();
		break;

	case CommMessageType::PropulsionExitManualControl:
		m_controller.exitManualControl();
		break;
	case CommMessageType::PropulsionSetControlLevels:
		{
			uint8_t buff[2];
			m_message_queue.pop_message((unsigned char*)buff,2);
			m_controller.setControlLevels(buff[0], buff[1]);
		}
		break;
	case CommMessageType::PropulsionSetTargetPose:
		{
			RobotPose pose;
			m_message_queue.pop_message((unsigned char*)&pose, sizeof(pose));
			m_controller.setTargetPose(pose);
		}
		break;
	case CommMessageType::PropulsionMeasureNormal:
			{
				float buff[2];
				m_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
				measureNormal(buff[0], buff[1]);
			}
			break;
	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}
}

void PropulsionTask::processUrgentMessage()
{
	auto message_type = (CommMessageType)m_urgent_message_queue.message_type();

	switch(message_type)
	{
	case CommMessageType::DbgPropulsionSetPose:
		{
			float pose[3];
			m_urgent_message_queue.pop_message((unsigned char*)&pose, 12);
			m_controller.resetPose(pose[0], pose[1], pose[2]);
		}
		break;
	case CommMessageType::PropulsionSetAdversaryDetectionEnable:
		{
			uint8_t buff;
			m_urgent_message_queue.pop_message((unsigned char*)&buff,1);
			m_adversary_detection_enabled = (bool)buff;
		}
		break;
	case CommMessageType::DbgGetOdometryConfig:
		{
			auto config = m_odometry.config();
			Robot::instance().mainExchangeOut().pushMessage(
					CommMessageType::DbgGetOdometryConfig,
					(unsigned char*)&config, sizeof(config));
			m_urgent_message_queue.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetOdometryConfig:
		{
			OdometryConfig config;
			m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
			m_odometry.setConfig(config);
		}
		break;
	case CommMessageType::DbgGetPropulsionConfig:
		{
			auto config = m_controller.config();
			Robot::instance().mainExchangeOut().pushMessage(
					CommMessageType::DbgGetPropulsionConfig,
					(unsigned char*)&config, sizeof(config));
			m_urgent_message_queue.pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetPropulsionConfig:
		{
			PropulsionControllerConfig config;
			m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
			m_controller.setConfig(config);
		}
		break;
	case CommMessageType::CmdEmergencyStop:
		m_controller.emergencyStop();
		m_urgent_message_queue.pop_message(nullptr, 0);
		break;
	case CommMessageType::PropulsionClearError:
			m_controller.clearError();
			m_urgent_message_queue.pop_message(nullptr, 0);
			break;
	case CommMessageType::PropulsionClearCommandQueue:
			m_urgent_message_queue.pop_message(nullptr, 0);
			while(m_message_queue.message_ready())
			{
				m_message_queue.pop_message(nullptr, 0);
			}
			break;
	case CommMessageType::DbgSetPropulsionEnable:
		{
			uint8_t enabled;
			m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
			m_controller.setEnable(enabled);
			if(!enabled)
			{
				// Clear queue on disable
				while(m_message_queue.message_ready())
				{
					m_message_queue.pop_message(nullptr, 0);
				}
			}
		}
		break;
	case CommMessageType::DbgSetMotorsEnable:
		{
			uint8_t enabled;
			m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
			Hal::set_motors_enable(enabled);
		}
		break;
	case CommMessageType::DbgSetMotorsPwm:
		{
			float pwm[2];
			m_urgent_message_queue.pop_message((unsigned char*)&pwm, 8);
			Hal::set_motors_pwm(pwm[0], pwm[1]);
		}
		break;
	case CommMessageType::PropulsionMeasurePoint:
		{
			float buff[4];
			m_urgent_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
			m_odometry.measurePerpendicularPoint(buff[0], buff[1], *(Vector2D*)(buff+2));
			auto pose = m_odometry.pose();
			// Set controller to new pose
			m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
		}
		break;
	default:
		m_urgent_message_queue.pop_message(nullptr, 0);
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

void PropulsionTask::measureNormal(float angle, float distance)
{
	auto pose = m_odometry.pose();
	Vector2D normal{cos(angle), sin(angle)};
	// Check if front or back is touching the border
	float dot = normal.x * cos(pose.yaw) + normal.y * sin(pose.yaw);
	if(dot >0)
	{
		// border normal is aligned with robot yaw
		// means the robot back is touching the border
		distance = distance + Robot::instance().robotConfig().back_length;
	} else
	{
		// touched on the front
		distance = distance + Robot::instance().robotConfig().front_length;
	}
	// Project current position on line and adjust yaw
	m_odometry.measureLineNormal(normal, distance);
	pose = m_odometry.pose();
	// Set controller to new pose
	m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
}

void PropulsionTask::taskFunction()
{
	// Register for messages
	Robot::instance().mainExchangeIn().subscribe({84,97, &m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({64,68, &m_urgent_message_queue});
	Robot::instance().mainExchangeIn().subscribe({80,83, &m_urgent_message_queue});
	Robot::instance().mainExchangeIn().subscribe({32,32, &m_urgent_message_queue});
	Robot::instance().mainExchangeIn().subscribe({98,102, &m_urgent_message_queue});

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
