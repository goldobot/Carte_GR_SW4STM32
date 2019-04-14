#include "goldobot/comm_manager.hpp"
#include "goldobot/robot.hpp"

#include <cstring>
#include "FreeRTOS.h"

#include "stm32f3xx.h"

using namespace goldobot;

CommManager::CommManager() :
	m_dbg_message_queue(m_dbg_message_queue_buffer, 512)
{
	m_dbg_message_queue_mutex = xSemaphoreCreateMutex();
}

bool CommManager::push_message(uint16_t message_type, const unsigned char* buffer, size_t size)
{
	bool retval = false;
	if(xSemaphoreTake(m_dbg_message_queue_mutex, portMAX_DELAY) == pdTRUE)
	{
		retval = m_dbg_message_queue.push_message(message_type, buffer, size);
		xSemaphoreGive(m_dbg_message_queue_mutex);
	}

	return retval;
}

void CommManager::pop_message(unsigned char* buffer, size_t size)
{
	if(xSemaphoreTake(m_dbg_message_queue_mutex, portMAX_DELAY) == pdTRUE)
	{
		m_dbg_message_queue.pop_message(buffer, size);
		xSemaphoreGive(m_dbg_message_queue_mutex);
	}
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
	case CommMessageType::DbgSetOdometryConfig:
			{
				OdometryConfig config;
				pop_message((unsigned char*)&config, sizeof(config));
				Robot::instance().odometry().setConfig(config);
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
	case CommMessageType::FpgaDbgReadReg:
		{
			unsigned int apb_data = 0xdeadbeef;
			unsigned char buff[8];
			pop_message(buff, 4);
			uint32_t apb_addr = *(uint32_t*)(buff);
			if(Robot::instance().fpgaTask().goldo_fpga_master_spi_read_word(apb_addr, &apb_data)!=0)
			{
				apb_data = 0xdeadbeef;
			}
			std::memcpy(buff+4, (unsigned char *)&apb_data, 4);
			comm.send_message(CommMessageType::FpgaDbgReadReg, (char *)buff, 8);
		}
		break;
	case CommMessageType::FpgaDbgWriteReg:
		{
			unsigned char buff[8];
			pop_message(buff, 8);
			uint32_t apb_addr = *(uint32_t*)(buff);
			uint32_t apb_data = *(uint32_t*)(buff+4);
			Robot::instance().fpgaTask().goldo_fpga_master_spi_write_word(apb_addr, apb_data);
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

void CommManager::on_msg_dbg_execute_trajectory()
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
	m_commands[id] = *(Command*)(buff+2);
}

void CommManager::on_msg_dbg_robot_set_point()
{
	unsigned char buff[10];
	pop_message(buff,10);
	uint16_t id = *(uint16_t*)(buff);
	m_waypoints[id] = *(Vector2D*)(buff+2);
}

void CommManager::on_msg_dbg_robot_set_trajectory_point()
{
	unsigned char buff[2];
	pop_message(buff,2);
	m_trajectory_points[buff[0]] = buff[1];
}

void CommManager::on_msg_dbg_robot_set_sequence()
{
	uint16_t buff[3];
	pop_message((unsigned char*)buff,6);
	m_sequences[buff[0]] = {buff[1], buff[2]};
}

void CommManager::on_msg_dbg_robot_execute_sequence()
{
	uint8_t seq_id;
	pop_message((unsigned char*)&seq_id,1);

	m_current_sequence_id = seq_id;
	m_current_command_id = m_sequences[m_current_sequence_id].begin_idx;
	m_sequence_active = true;
	m_wait_current_cmd = false;
}



