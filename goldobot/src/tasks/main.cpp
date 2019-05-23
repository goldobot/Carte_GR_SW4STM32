#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "stm32f3xx.h"
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include <cmath>

using namespace goldobot;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

int g_foo;
MainTask::MainTask():
    m_match_state(State::Idle),
    m_dbg_message_queue(m_dbg_message_queue_buffer, 512),
    m_sequences_initialized(false),
    m_sequence_active(false)
{
	m_dbg_message_queue_mutex = xSemaphoreCreateMutex();
#if 0 /* FIXME : DEBUG */
	m_speed_settings[0] = {0.5,1,1,2,1,1,0.2};
	m_speed_settings[1] = {0.25,0.5,0.5,1,0.5,0.5,0.2};
#else
	m_speed_settings[0] = {0.33,0.66,0.66,1,0.5,0.5,0.2};
	m_speed_settings[1] = {0.125,0.25,0.25,0.5,0.25,0.25,0.1};
#endif
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
	if (!m_sequences_initialized) return;

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

void MainTask::execute_sequence(int seq_id)
{
	auto& comm = Robot::instance().comm();
	m_current_sequence_id = seq_id;
	m_current_command_id = m_sequences[m_current_sequence_id].begin_idx;
	m_sequence_active = true;
	m_wait_current_cmd = false;
	comm.send_debug_event(DbgEventType::DbgEventStartSequence, (uint32_t)seq_id, 0, 0);
}
void MainTask::sequence_step()
{
	if(!m_sequence_active)
	{
		return;
	}

	auto& arms = Robot::instance().arms();
	auto& propulsion = Robot::instance().propulsion();
	auto& cmd = m_commands[m_current_command_id];

	//exec
	if(!m_wait_current_cmd)
	{
		m_wait_current_cmd = _execute_command(cmd) && cmd.blocking;
	}

	// waiting
	if(m_wait_current_cmd)
	{
		m_wait_current_cmd = !_current_command_finished(cmd);
	}

	if(!m_wait_current_cmd)
	{
		m_current_command_id++;
		if (m_current_command_id == m_sequences[m_current_sequence_id].end_idx)
		{
			m_sequence_active = false;
		}
	}
}
bool MainTask::_execute_command(const Command& cmd)
{
	auto& comm = Robot::instance().comm();

	float delta_yaw=0;
	auto& arms = Robot::instance().arms();
	auto& propulsion = Robot::instance().propulsion();

	switch(cmd.opcode)
	{
	case OpCode::Delay:
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.delay_ms, 0);

		m_delay_finished_ts = xTaskGetTickCount() + cmd.delay_ms;
		return true;
	case OpCode::SetPose:
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)(m_waypoints[cmd.set_pose.pose_id].x*1000.0), (uint32_t)(m_waypoints[cmd.set_pose.pose_id].y*1000.0));

		propulsion.reset_pose(m_waypoints[cmd.set_pose.pose_id].x, m_waypoints[cmd.set_pose.pose_id].y, cmd.set_pose.angle_deg * M_PI / 180);
		return false;
	case OpCode::Rotation:
	{
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.rotation.angle, 0);

		delta_yaw = cmd.rotation.angle * M_PI / 180;
		auto& ss = m_speed_settings[cmd.trajectory.speed_settings];
		propulsion.executeRotation(delta_yaw,ss.yaw_rate,ss.yaw_acceleration, ss.yaw_decceleration);
		return true;
	}
	case OpCode::Reposition:
	{
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.reposition.angle, 0);

		auto dir = PropulsionController::Direction::Forward;
		float angle_rad = cmd.reposition.angle * M_PI/180;
		Vector2D normal = {cosf(angle_rad), sinf(angle_rad)};
		// Check angle of current robot position compared to border
		auto pose = Robot::instance().odometry().pose();
		float ux = cos(pose.yaw);
		float uy = sinf(pose.yaw);
		float dist_m = cmd.reposition.distance*1e-3;

		if(ux * normal.x + uy * normal.y >= 0)
		{
			dir = PropulsionController::Direction::Forward;
			dist_m -= Robot::instance().robotConfig().front_length;
		}
		else
		{
			dir = PropulsionController::Direction::Backward;
			dist_m -= Robot::instance().robotConfig().back_length;
		}

		propulsion.executeRepositioning(dir, 0.2,normal,dist_m);
		return true;
	}
	case OpCode::Trajectory:
	{
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.trajectory.num_points, 0);

		Vector2D points[16];
		for(unsigned i=0;i < cmd.trajectory.num_points;i++)
		{
			int pt_id = m_trajectory_points[cmd.trajectory.begin_idx + i];
			points[i] = m_waypoints[pt_id];
			comm.send_debug_event(DbgEventType::DbgEventGoWaypoint, (uint32_t)(points[i].x*1000.0), (uint32_t)(points[i].y*1000.0), 0);
		}
		auto& ss = m_speed_settings[cmd.trajectory.speed_settings];
		propulsion.executeTrajectory(points,cmd.trajectory.num_points,ss.speed, ss.acceleration, ss.decceleration);
		return true;
	}
	case OpCode::ArmsGoToPosition:
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.arms.arm_id, (uint32_t)cmd.arms.seq_or_pos_id);

		arms.go_to_position(cmd.arms.arm_id, cmd.arms.seq_or_pos_id,0);
		return true;
		break;
	case OpCode::ArmsExecuteSequence:
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, (uint32_t)cmd.arms.arm_id, (uint32_t)cmd.arms.seq_or_pos_id);

		arms.execute_sequence(cmd.arms.arm_id, cmd.arms.seq_or_pos_id);
		return true;
	default:
		comm.send_debug_event(DbgEventType::DbgEventExecuteCommand, (uint32_t)cmd.opcode, 0, 0);

		return false;
	}
}
bool MainTask::_current_command_finished(const Command& cmd)
{
	auto& arms = Robot::instance().arms();
	auto& propulsion = Robot::instance().propulsion();

	switch(cmd.opcode)
	{
	case OpCode::Delay:
		if(xTaskGetTickCount() >= m_delay_finished_ts)
		{
			return true;
		}
		break;
	case OpCode::Rotation:
		if(propulsion.state() == PropulsionController::State::Stopped)
		{
			return true;
		}
		break;
	case OpCode::Reposition:
		if(propulsion.state() == PropulsionController::State::Stopped)
		{
			return true;
		}
		break;
	case OpCode::Trajectory:
		if(propulsion.state() == PropulsionController::State::Stopped)
		{
			return true;
		}
		break;
	case OpCode::ArmsGoToPosition:
		return true;
		break;
	case OpCode::ArmsExecuteSequence:
		return !arms.m_arms_moving[cmd.arms.arm_id];
		break;
	default:
		break;
	}
	return false;
}

void MainTask::taskFunction()
{
#if 0 /* FIXME : TODO : remove (old 2018) */
	m_trajectory_planner.add_point(0.24, -0.65);
	m_trajectory_planner.add_point(0.3, -0.65);
	m_trajectory_planner.add_point(0.338, -0.65);// cube pos

	m_trajectory_planner.add_point(0.938, -1.2);// Second cube on green side

	m_trajectory_planner.add_edge(0,1);
	m_trajectory_planner.add_edge(1,2);
	m_trajectory_planner.add_edge(2,3);

	m_trajectory_planner.add_edge(2,4);
	m_trajectory_planner.compile();
#endif

	Robot::instance().propulsion().reset_pose(0.75, -1.22, M_PI/2);

	m_trajectory_planner.add_point(0.75, -1.22);
	m_trajectory_planner.add_point(0.75, -1.0);
	m_trajectory_planner.add_edge(0,1);
	m_trajectory_planner.compile();

	auto& comm = Robot::instance().comm();

	while(1)
	{

		uint32_t clock = xTaskGetTickCount();
		process_messages();
		sequence_step();

		switch(m_match_state)
		{
		case State::Idle:
			if((m_sequences_initialized) && (Hal::get_gpio(1)))
			{
				if( Hal::get_gpio(4))
				{
					m_side = Side::Green;
				} else
				{
					m_side = Side::Orange;
				}

				comm.send_debug_event(DbgEventType::DbgEventRobotHoming, (uint32_t)m_side, 0, 0);

				switch(m_side)
				{
				case Side::Green:
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 32000);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 32000);
					break;
				case Side::Orange:
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 45000);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 45000);
					break;
				default:
					break;
				}

				m_match_state = State::WaitForStartOfMatch;
				Hal::set_motors_enable(true);
				Robot::instance().propulsion().enable();
				switch(m_side)
				{
				case Side::Green:
					execute_sequence(0);
					break;
				case Side::Orange:
					execute_sequence(1);
					break;
				default:
					break;
				}
			}
			break;
		case State::WaitForStartOfMatch:
			if((m_sequences_initialized) && (!Hal::get_gpio(1)))
			{
				{
					auto& comm = Robot::instance().comm();
					uint32_t clock = xTaskGetTickCount();
					comm.send_message(CommMessageType::StartOfMatch,(char*)&clock,sizeof(clock));
					m_start_of_match_time = clock;
					Hal::set_gpio(0, true);
				}

				switch(m_side)
				{
				case Side::Green:
					execute_sequence(2);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 32000);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 32000);
					m_match_state = State::Match;
					break;
				case Side::Orange:
					execute_sequence(3);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 45000);
					//Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 45000);
					m_match_state = State::Match;
					break;
				default:
					break;
				}

				comm.send_debug_event(DbgEventType::DbgEventStartMatch, (uint32_t)m_side, 0, 0);
			}
			break;

		case State::Match:
			{
#if 0 /* FIXME : DEBUG */
				if(remainingMatchTime() == 80)
				{
					switch(m_side)
					{
						case Side::Green:
							Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 45000);
							break;
						case Side::Orange:
							Robot::instance().fpgaTask().goldo_fpga_cmd_servo(10, 32000);
							break;
						default:
							break;
					}
				}
#endif

				if(remainingMatchTime() == 0)
				{
					Hal::set_gpio(0, false);
					m_match_state = State::PostMatch;
					comm.send_message(CommMessageType::EndOfMatch,(char*)&clock,sizeof(clock));
					Hal::set_motors_enable(false);
				}
			}
			break;
		case State::PostMatch:
			break;
		case State::ManualControl:
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
		// Reset the FPGA (soft reset, only robot IPs..)
		Robot::instance().fpgaTask().goldo_fpga_master_spi_write_word (0x800084f0, 42);
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
	case CommMessageType::DbgRobotEnterManualMode:
		pop_message(nullptr, 0);
		m_match_state = State::ManualControl;
		m_sequence_active = false;
		m_wait_current_cmd = false;
		Hal::set_motors_enable(false);
		Robot::instance().propulsion().disable();
		Robot::instance().propulsion().reset_pose(0.75, -1.22, M_PI/2);
		break;
	case CommMessageType::DbgRobotExitManualMode:
		pop_message(nullptr, 0);
		m_match_state = State::Idle;
		m_sequence_active = false;
		m_wait_current_cmd = false;
		Hal::set_motors_enable(false);
		Robot::instance().propulsion().disable();
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

void MainTask::on_msg_dbg_arms_set_pose()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[12];
	pop_message(buff,12);

	auto& arm_descr = arms.m_arms_descrs[buff[0]];

	uint16_t* ptr = arms.m_arms_positions+arm_descr.positions_idx_begin + buff[1]*arm_descr.num_servos;
	memcpy(ptr, buff+2, arm_descr.num_servos * 2);
}

void MainTask::on_msg_dbg_arms_set_command()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[7];
	pop_message(buff,7);
	memcpy(arms.m_arms_commands+buff[0], buff+1, 6);
}

void MainTask::on_msg_dbg_arms_set_sequences(uint16_t message_size)
{
	auto& arms = Robot::instance().arms();
	pop_message((unsigned char*)(arms.m_arms_sequences), message_size);
}

void MainTask::on_msg_dbg_arms_go_to_position()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[2];
	pop_message(buff,2);
	arms.go_to_position(buff[0], buff[1], 2000, 0);
}
void MainTask::on_msg_dbg_arms_execute_sequence()
{
	auto& arms = Robot::instance().arms();
	unsigned char buff[2];
	pop_message(buff,2);
	arms.execute_sequence(buff[0], buff[1]);
}

void MainTask::on_msg_dbg_robot_set_command()
{
	unsigned char buff[12];
	pop_message(buff,12);
	uint16_t id = *(uint16_t*)(buff);
	m_commands[id] = *(Command*)(buff+2);
}

void MainTask::on_msg_dbg_robot_set_point()
{
	unsigned char buff[10];
	pop_message(buff,10);
	uint16_t id = *(uint16_t*)(buff);
	m_waypoints[id] = *(Vector2D*)(buff+2);
}

void MainTask::on_msg_dbg_robot_set_trajectory_point()
{
	unsigned char buff[2];
	pop_message(buff,2);
	m_trajectory_points[buff[0]] = buff[1];
}

void MainTask::on_msg_dbg_robot_set_sequence()
{
	uint16_t buff[3];
	pop_message((unsigned char*)buff,6);
	m_sequences[buff[0]] = {buff[1], buff[2]};
	m_sequences_initialized = true; /* FIXME : TODO : un peu moins brutal.. */
}

void MainTask::on_msg_dbg_robot_execute_sequence()
{
	uint8_t seq_id;
	pop_message((unsigned char*)&seq_id,1);

	if (!m_sequences_initialized) return;

	if (m_match_state != State::ManualControl) return;

	m_current_sequence_id = seq_id;
	m_current_command_id = m_sequences[m_current_sequence_id].begin_idx;
	m_sequence_active = true;
	m_wait_current_cmd = false;

	Hal::set_motors_enable(true);
	Robot::instance().propulsion().enable();
}




