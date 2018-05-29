#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "stm32f3xx.h"
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

int g_foo;
MainTask::MainTask():
    m_match_state(State::Idle),
	m_dbg_message_queue(m_dbg_message_queue_buffer, 512)
{
	m_dbg_message_queue_mutex = xSemaphoreCreateMutex();;
}

const char* MainTask::name() const
{
	return "main";
}

int MainTask::remainingMatchTime()
{
	int elapsed_time = (xTaskGetTickCount() - m_start_of_match_time)/1000;
	return elapsed_time < 90 ? 90 - elapsed_time : 0;
}

void MainTask::preMatchBegin()
{

}

void MainTask::preMatchStep()
{

}

void MainTask::matchBegin()
{
	auto& comm = Robot::instance().comm();
	uint32_t clock = xTaskGetTickCount();
	comm.send_message(CommMessageType::StartOfMatch,(char*)&clock,sizeof(clock));
	m_match_state = State::Match;
	m_start_of_match_time = clock;
	Hal::set_gpio(0, true);
	Hal::set_motors_enable(true);
	auto& prop = Robot::instance().propulsion();
	prop.test = true;
	Robot::instance().propulsion().reset_pose(0, 0, 0);
	m_current_objective = 0;
	return;
	//matchSelectNextObjective();

	// Dirty
	Robot::instance().propulsion().reset_pose(0.25, -1.28, -M_PI/2);
	m_current_objective = 0;
	matchSelectNextObjective();
}


void MainTask::matchStep()
{
	auto& prop = Robot::instance().propulsion();

	if(m_current_objective == 0)
	{
		Vector2D points[2] = {
				{0,0},
				{1,0}
		};

		prop.executeTrajectory(points, 2, 0.5,1,1);
		m_current_objective = 1;
	}
}

void MainTask::matchSelectNextObjective()
{
	m_trajectory_planner.set_current_point(m_current_objective);
	switch(m_current_objective)
	{
	case 0:
		m_current_objective = 3;
		break;
	case 3:
		m_current_objective = 4;
		break;
	case 4:
		m_current_objective = 3;
		break;
	}
	m_current_trajectory_index = 0;
	m_trajectory_planner.compute_costs();
	m_trajectory_planner.compute_trajectory(m_current_objective);
}

void MainTask::taskFunction()
{
	// Dirty. init points

	m_trajectory_planner.add_point(0.24, -1.28);// 1: green starting position
	m_trajectory_planner.add_point(0.24, -0.65);
	m_trajectory_planner.add_point(0.3, -0.65);
	m_trajectory_planner.add_point(0.338, -0.65);// cube pos

	m_trajectory_planner.add_point(0.938, -1.2);// Second cube on green side

	m_trajectory_planner.add_edge(0,1);
	m_trajectory_planner.add_edge(1,2);
	m_trajectory_planner.add_edge(2,3);

	m_trajectory_planner.add_edge(2,4);
	m_trajectory_planner.compile();

	auto& comm = Robot::instance().comm();


	while(1)
	{
		uint32_t clock = xTaskGetTickCount();
		process_messages();

		switch(m_match_state)
		{
		case State::Idle:
			if(!Hal::get_gpio(1))
			{
				m_match_state = State::WaitForStartOfMatch;
			}

			break;
		case State::WaitForStartOfMatch:
			if(Hal::get_gpio(1))
			{
				matchBegin();
			}
			break;

		case State::Match:
			{
				if(remainingMatchTime() < 70)
				{
					Hal::set_gpio(0, false);
					m_match_state = State::PostMatch;
					comm.send_message(CommMessageType::EndOfMatch,(char*)&clock,sizeof(clock));
					Hal::set_motors_enable(false);
				} else
				{
					matchStep();
				}
			}
			break;
		case State::PostMatch:

			//m_match_state = State::Idle;
			break;
		}
		vTaskDelay(1);
	}
}

bool MainTask::push_message(uint16_t message_type, const unsigned char* buffer, size_t size)
{
	bool retval = false;
	if(xSemaphoreTake(m_dbg_message_queue_mutex, portMAX_DELAY) == pdTRUE)
	{
		retval = m_dbg_message_queue.push_message(message_type, buffer, size);
		xSemaphoreGive(m_dbg_message_queue_mutex);
	}

	return retval;
}

void MainTask::pop_message(unsigned char* buffer, size_t size)
{
	if(xSemaphoreTake(m_dbg_message_queue_mutex, portMAX_DELAY) == pdTRUE)
	{
		m_dbg_message_queue.pop_message(buffer, size);
		xSemaphoreGive(m_dbg_message_queue_mutex);
	}
}

void MainTask::process_messages()
{
	while(m_dbg_message_queue.message_ready())
	{
		process_message((CommMessageType)m_dbg_message_queue.message_type(), m_dbg_message_queue.message_size());
	}
}

void MainTask::process_message(CommMessageType message_type, uint16_t message_size)
{
	auto& comm = Robot::instance().comm();
	goldobot::PropulsionController* propulsion = &(Robot::instance().propulsion());

	switch(message_type)
	{
	case CommMessageType::DbgGetOdometryConfig:
		{
			auto config = Robot::instance().odometry().config();
			comm.send_message(CommMessageType::DbgGetOdometryConfig, (char*)&config, sizeof(config));
			pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgGetPropulsionConfig:
		{
			auto config = Robot::instance().propulsion().config();
			comm.send_message(CommMessageType::DbgGetPropulsionConfig, (char*)&config, sizeof(config));
			pop_message(nullptr, 0);
		}
		break;
	case CommMessageType::DbgSetPropulsionConfig:
		{
			PropulsionControllerConfig config;
			pop_message((unsigned char*)&config, sizeof(config));
			propulsion->set_config(config);
		}
		break;
	case CommMessageType::DbgReset:
		// Reset the micro
		NVIC_SystemReset();
		break;
	case CommMessageType::CmdEmergencyStop:
		propulsion->emergency_stop();
		pop_message(nullptr, 0);
		break;
	case CommMessageType::DbgSetPropulsionEnable:
		{
			uint8_t enabled;
			pop_message((unsigned char*)&enabled, 1);
			if(enabled)
			{
				propulsion->enable();
			} else
			{
				propulsion->disable();
			}
		}
		break;
	case CommMessageType::DbgSetMotorsEnable:
		{
			uint8_t enabled;
			pop_message((unsigned char*)&enabled, 1);
			Hal::set_motors_enable(enabled);
		}
		break;
	case CommMessageType::DbgSetMotorsPwm:
		{
			float pwm[2];
			pop_message((unsigned char*)&pwm, 8);
			Hal::set_motors_pwm(pwm[0], pwm[1]);
		}
		break;
	case CommMessageType::DbgPropulsionTest:
		{
			uint8_t id;
			pop_message((unsigned char*)&id, 1);
			uint8_t status = 0;
			comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);

			switch(id)
			{
			case 0:
				propulsion->executeTest(PropulsionController::TestPattern::SpeedSteps);
				break;
			}
			while(propulsion->state() == PropulsionController::State::Test)
			{
				delay(1);
			}
			status = 1;
			comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);
		}
		break;
	case CommMessageType::DbgDynamixelsList:
		{
			pop_message(nullptr, 0);
			uint8_t buff[4] = {25,1};
			for(unsigned id = 0; id < 0xFE; id++)
			{
				if(Robot::instance().arms().dynamixels_read_data(id,0 , buff, 4))
				{
					comm.send_message(CommMessageType::DbgDynamixelDescr, (char*)buff, 4);
				}
			}
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueEnable:
		{
			unsigned char buff[2];
			pop_message(buff, 2);
			// Torque enable
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x18, buff+1, 1);
		}
		break;
	case CommMessageType::DbgDynamixelSetGoalPosition:
		{
			unsigned char buff[3];
			pop_message(buff, 3);
			// Goal position
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x1E, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelSetTorqueLimit:
		{
			unsigned char buff[3];
			pop_message(buff, 3);
			// Goal position
			Robot::instance().arms().dynamixels_write_data(buff[0], 0x22, buff+1, 2);
		}
		break;
	case CommMessageType::DbgDynamixelGetRegisters:
			{
				unsigned char buff[3];
				unsigned char data_read[64];

				pop_message(buff, 3);
				std::memcpy(data_read, buff, 2);
				if(Robot::instance().arms().dynamixels_read_data(buff[0], buff[1], data_read+2, buff[2]))
				{
					comm.send_message(CommMessageType::DbgDynamixelGetRegisters, (char*)data_read, buff[2]+2);
				}
			}
			break;
	case CommMessageType::DbgDynamixelSetRegisters:
		{
			unsigned char buff[128];
			uint16_t size = message_size;
			pop_message(buff, 128);
			//id, addr, data
			if(Robot::instance().arms().dynamixels_write_data(buff[0], buff[1], buff+2, size-2));

		}
		break;
	case CommMessageType::DbgPropulsionSetPose:
		{
			float pose[3];
			pop_message((unsigned char*)&pose, 12);
			Robot::instance().propulsion().reset_pose(pose[0], pose[1], pose[2]);
		}
		break;
	case CommMessageType::DbgPropulsionExecuteReposition:
		{
			unsigned char buff[17];
			pop_message(buff, 17);
			int8_t dir = *(int8_t*)(buff);
			float speed = *(float*)(buff+1);
			Vector2D normal = *(Vector2D*)(buff+5);
			float distance_to_center = *(float*)(buff+13);

			if(dir == 1)
			{
				Robot::instance().propulsion().executeRepositioning(
						PropulsionController::Direction::Forward,
						speed,
						normal,
						distance_to_center - Robot::instance().robotConfig().front_length);
			}


		}
		break;
	case CommMessageType::DbgPropulsionExecuteTrajectory:
		on_msg_dbg_execute_trajectory();
		break;

	default:
		pop_message(nullptr, 0);
		break;
	}
}

void MainTask::on_msg_dbg_execute_trajectory()
{
	auto& comm = Robot::instance().comm();
	auto& propulsion = Robot::instance().propulsion();

	unsigned char buff[14];
	pop_message(buff,14);
	uint8_t pattern = buff[0];
	int8_t direction = buff[0];
	float speed = *(float*)(buff+2);
	float accel = *(float*)(buff+6);
	float deccel = *(float*)(buff+10);
	propulsion.reset_pose(0, 0, 0);
	uint8_t status = 0;
	comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);

	switch(pattern)
	{
	case 0:
		{
			Vector2D points[2] = {{0,0}, {0.5,0}};
			propulsion.executeTrajectory(points,2,speed, accel, deccel);
		}
		break;
	case 1:
		{
			Vector2D points[2] = {{0,0}, {-0.5,0}};
			propulsion.executeTrajectory(points,2,speed, accel, deccel);
		}
		break;
	case 2:
		{
			Vector2D points[3] = {{0,0}, {0.5,0}, {0.5,0.5}};
			propulsion.executeTrajectory(points,3,speed, accel, deccel);
		}
		break;
	case 3:
		{
			Vector2D points[3] = {{0,0}, {-0.5,0}, {-0.5,-0.5}};
			propulsion.executeTrajectory(points,3,speed, accel, deccel);
		}
		break;
	case 4:
		{
			Vector2D points[4] = {{0,0}, {0.5,0}, {0.5,0.5}, {1,0.5} };
			propulsion.executeTrajectory(points,4,speed, accel, deccel);
		}
		break;
	}
	while(propulsion.state() == PropulsionController::State::FollowTrajectory)
	{
		delay(1);
	}
	status = 1;
	comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);
}


