#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

int g_foo;
MainTask::MainTask():
    m_match_state(State::Idle)
{
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
	comm.send_message((uint16_t)CommMessageType::StartOfMatch,(char*)&clock,sizeof(clock));
	m_match_state = State::Match;
	m_start_of_match_time = clock;
	Hal::set_gpio(0, true);

	// Dirty
	Robot::instance().propulsion().reset_pose(0.25, -1.32, -M_PI/2);
	m_current_objective = 0;
	matchSelectNextObjective();
}


void MainTask::matchStep()
{
	auto& prop = Robot::instance().propulsion();

	if(m_current_trajectory_index + 1 == m_trajectory_planner.num_trajectory_points())
	{
		matchSelectNextObjective();
		return;
	}
	prop.executePointTo(m_trajectory_planner.trajectory_point(m_current_trajectory_index+1), 1,1,1);
	while(prop.state() != PropulsionController::State::Stopped)
	{
		delayTicks(1);
	}
	Vector2D points[2];
	points[0] = m_trajectory_planner.trajectory_point(m_current_trajectory_index);
	points[1] = m_trajectory_planner.trajectory_point(m_current_trajectory_index+1);
	prop.executeTrajectory(points, 2, 0.5,1,1);

	while(prop.state() != PropulsionController::State::Stopped)
	{
		delayTicks(1);
	}
	m_current_trajectory_index++;
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

	m_trajectory_planner.add_point(0.24, -1.32);// 1: green starting position
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

		switch(m_match_state)
		{
		case State::Idle:
			m_match_state = State::WaitForStartOfMatch;
			break;
		case State::WaitForStartOfMatch:
			if(!Hal::get_gpio(1))
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
					comm.send_message((uint16_t)CommMessageType::EndOfMatch,(char*)&clock,sizeof(clock));
				} else
				{
					matchStep();
				}
			}
			break;
		case State::PostMatch:
			m_match_state = State::Idle;
			break;
		}
		vTaskDelay(1);
	}
}
