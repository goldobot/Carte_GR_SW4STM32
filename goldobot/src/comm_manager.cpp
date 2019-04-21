#include "goldobot/comm_manager.hpp"
#include "goldobot/robot.hpp"

#include <cstring>
#include "FreeRTOS.h"

#include "stm32f3xx.h"

using namespace goldobot;

CommManager::CommManager() :
	m_dbg_message_queue(m_dbg_message_queue_buffer, 512)
{
}

bool CommManager::push_message(uint16_t message_type, const unsigned char* buffer, size_t size)
{
	return m_dbg_message_queue.push_message(message_type, buffer, size);
}

void CommManager::pop_message(unsigned char* buffer, size_t size)
{
	m_dbg_message_queue.pop_message(buffer, size);
}

void CommManager::process_messages()
{
	while(m_dbg_message_queue.message_ready())
	{
		process_message((CommMessageType)m_dbg_message_queue.message_type(), m_dbg_message_queue.message_size());
	}
}

void CommManager::process_message(CommMessageType message_type, uint16_t message_size)
{
	//auto& comm = Robot::instance().comm();
	/*
	switch(message_type)
	{

	case CommMessageType::DbgReset:
		// Reset the micro
		NVIC_SystemReset();
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
			case 1:
				propulsion->executeTest(PropulsionController::TestPattern::YawRateSteps);
				break;
			case 2:
				propulsion->executeTest(PropulsionController::TestPattern::PositionStaticSteps);
				break;
			case 3:
				propulsion->executeTest(PropulsionController::TestPattern::YawSteps);
				break;
			}
			while(propulsion->state() == PropulsionController::State::Test)
			{
				//delay(1);
			}
			status = 1;
			comm.send_message(CommMessageType::DbgPropulsionExecuteTrajectory, (char*)&status, 1);
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

	case CommMessageType::FpgaCmdDCMotor:
		{
			unsigned char buff[3];
			pop_message(buff, 3);
			int motor_id = buff[0];
			int pwm = *(int16_t*)(buff+1);
			Robot::instance().fpgaTask().goldo_fpga_cmd_motor(motor_id, pwm);
		}
		break;
	case CommMessageType::FpgaCmdServo:
		{
			unsigned char buff[5];
			pop_message(buff, 5);
			int motor_id = buff[0];
			int pwm = *(uint32_t*)(buff+1);
			Robot::instance().fpgaTask().goldo_fpga_cmd_servo(motor_id, pwm);
		}
		break;
	case CommMessageType::FpgaColumnsCalib:
		{
			unsigned char buff[1];
			pop_message(buff, 1);
			Robot::instance().fpgaTask().goldo_fpga_columns_calib();
		}
		break;
	case CommMessageType::FpgaColumnsMove:
		{
			unsigned char buff[1];
			pop_message(buff, 1);
			int columns_pos = buff[0];
			Robot::instance().fpgaTask().goldo_fpga_columns_move(columns_pos);
		}
		break;
	case CommMessageType::FpgaColumnsSetOffset:
		{
			unsigned char buff[5];
			pop_message(buff, 5);
			int column_id = buff[0];
			int column_offset = *(uint32_t*)(buff+1);
			Robot::instance().fpgaTask().goldo_fpga_set_columns_offset(column_id, column_offset);
		}
		break;
	case CommMessageType::DbgArmsSetPose:
		on_msg_dbg_arms_set_pose();
		break;
	case CommMessageType::DbgArmsSetCommand:
		on_msg_dbg_arms_set_command();
		break;
	case CommMessageType::DbgArmsSetSequences:
		on_msg_dbg_arms_set_sequences(message_size);
		break;
	case CommMessageType::DbgArmsGoToPosition:
		on_msg_dbg_arms_go_to_position();
		break;
	case CommMessageType::DbgArmsExecuteSequence:
		on_msg_dbg_arms_execute_sequence();
		break;
	case CommMessageType::DbgRobotSetCommand:
		on_msg_dbg_robot_set_command();
		break;
	case CommMessageType::DbgRobotSetPoint:
		on_msg_dbg_robot_set_point();
		break;
	case CommMessageType::DbgRobotSetSequence:
		on_msg_dbg_robot_set_sequence();
		break;
	case CommMessageType::DbgRobotExecuteSequence:
		on_msg_dbg_robot_execute_sequence();
		break;
	case CommMessageType::DbgRobotSetTrajectoryPoint:
		on_msg_dbg_robot_set_trajectory_point();
		break;

	default:
		pop_message(nullptr, 0);
		break;
	}
}

void CommManager::on_msg_dbg_arms_set_pose()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[12];
	pop_message(buff,12);

	auto& arm_descr = arms.m_arms_descrs[buff[0]];

	uint16_t* ptr = arms.m_arms_positions+arm_descr.positions_idx_begin + buff[1]*arm_descr.num_servos;
	memcpy(ptr, buff+2, arm_descr.num_servos * 2);
}

void CommManager::on_msg_dbg_arms_set_command()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[7];
	pop_message(buff,7);
	memcpy(arms.m_arms_commands+buff[0], buff+1, 6);
}

void CommManager::on_msg_dbg_arms_set_sequences(uint16_t message_size)
{
	auto& arms = Robot::instance().arms();
	pop_message((unsigned char*)(arms.m_arms_sequences), message_size);
}

void CommManager::on_msg_dbg_arms_go_to_position()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[2];
	pop_message(buff,2);
	arms.go_to_position(buff[0], buff[1], 2000, 0);
}
void CommManager::on_msg_dbg_arms_execute_sequence()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[2];
	pop_message(buff,2);
	arms.execute_sequence(buff[0], buff[1]);
}

void CommManager::on_msg_dbg_robot_set_command()
{
	unsigned char buff[12];
	pop_message(buff,12);
	uint16_t id = *(uint16_t*)(buff);
	//m_commands[id] = *(Command*)(buff+2);
}

void CommManager::on_msg_dbg_robot_set_point()
{
	unsigned char buff[10];
	pop_message(buff,10);
	uint16_t id = *(uint16_t*)(buff);
	//m_waypoints[id] = *(Vector2D*)(buff+2);
}

void CommManager::on_msg_dbg_robot_set_trajectory_point()
{
	unsigned char buff[2];
	pop_message(buff,2);
	//m_trajectory_points[buff[0]] = buff[1];
}

void CommManager::on_msg_dbg_robot_set_sequence()
{
	uint16_t buff[3];
	pop_message((unsigned char*)buff,6);
	//m_sequences[buff[0]] = {buff[1], buff[2]};
}

void CommManager::on_msg_dbg_robot_execute_sequence()
{
	uint8_t seq_id;
	pop_message((unsigned char*)&seq_id,1);

	//m_current_sequence_id = seq_id;
	//m_current_command_id = m_sequences[m_current_sequence_id].begin_idx;
	//m_sequence_active = true;
	//m_wait_current_cmd = false;
}
*/
}

