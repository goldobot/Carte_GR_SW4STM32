#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

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
    m_match_state(State::Idle)
{
	m_speed_settings[0] = {0.5,1,1,2,1,1,0.2};
	m_speed_settings[1] = {0.25,0.5,0.5,1,0.5,0.5,0.2};
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
	auto& exchange = Robot::instance().mainExchangeOut();
	uint32_t clock = xTaskGetTickCount();
	exchange.pushMessage(CommMessageType::StartOfMatch,(unsigned char*)&clock,sizeof(clock));
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
	m_current_sequence_id = seq_id;
	m_current_command_id = m_sequences[m_current_sequence_id].begin_idx;
	m_sequence_active = true;
	m_wait_current_cmd = false;
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
	float delta_yaw=0;
	auto& arms = Robot::instance().arms();
	auto& propulsion = Robot::instance().propulsion();

	switch(cmd.opcode)
	{
	case OpCode::Delay:
		m_delay_finished_ts = xTaskGetTickCount() + cmd.delay_ms;
		return true;
	case OpCode::SetPose:
		propulsion.reset_pose(m_waypoints[cmd.set_pose.pose_id].x, m_waypoints[cmd.set_pose.pose_id].y, cmd.set_pose.angle_deg * M_PI / 180);
		return false;
	case OpCode::Rotation:
	{
		delta_yaw = cmd.rotation.angle * M_PI / 180;
		auto& ss = m_speed_settings[cmd.trajectory.speed_settings];
		propulsion.executeRotation(delta_yaw,ss.yaw_rate,ss.yaw_acceleration, ss.yaw_decceleration);
		return true;
	}
	case OpCode::Reposition:
		{
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
			} else
			{
				dir = PropulsionController::Direction::Backward;
				dist_m -= Robot::instance().robotConfig().back_length;
			}

			propulsion.executeRepositioning(dir, 0.2,normal,dist_m);
			return true;
		}
	case OpCode::Trajectory:
	{
		Vector2D points[16];
		for(unsigned i=0;i < cmd.trajectory.num_points;i++)
		{
			int pt_id = m_trajectory_points[cmd.trajectory.begin_idx + i];
			points[i] = m_waypoints[pt_id];
		}
		auto& ss = m_speed_settings[cmd.trajectory.speed_settings];
		propulsion.executeTrajectory(points,cmd.trajectory.num_points,ss.speed, ss.acceleration, ss.decceleration);
		return true;
	}
	case OpCode::ArmsGoToPosition:
		arms.go_to_position(cmd.arms.arm_id, cmd.arms.seq_or_pos_id,0);
		return true;
		break;
	case OpCode::ArmsExecuteSequence:
		arms.execute_sequence(cmd.arms.arm_id, cmd.arms.seq_or_pos_id);
		return true;
	default:
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

	//auto& comm = Robot::instance().comm();


	while(1)
	{

		uint32_t clock = xTaskGetTickCount();
		sequence_step();

		switch(m_match_state)
		{
		case State::Idle:
			if(!Hal::get_gpio(1))
			{
				if( Hal::get_gpio(4))
				{
					m_side = Side::Green;
				} else
				{
					m_side = Side::Orange;
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
			if(Hal::get_gpio(1))
			{
				switch(m_side)
				{
				case Side::Green:
					execute_sequence(2);
					m_match_state = State::Match;
					break;
				case Side::Orange:
					execute_sequence(3);
					m_match_state = State::Match;
					break;
				default:
					break;
				}
			}
			break;

		case State::Match:
			{
				if(remainingMatchTime() == 0)
				{
					Hal::set_gpio(0, false);
					m_match_state = State::PostMatch;
					//comm.send_message(CommMessageType::EndOfMatch,(char*)&clock,sizeof(clock));
					Hal::set_motors_enable(false);
				}
			}
			break;
		case State::PostMatch:
			break;
		}
		vTaskDelay(1);
	}
}


