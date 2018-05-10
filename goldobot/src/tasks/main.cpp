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
	Robot::instance().propulsion().reset_pose(0.2, -1.32, -M_PI/2);
	g_foo = 0;
}

Vector2D allpoints[] =
{
	{0.2, -1.32},
	{0.2, -0.65},
	{0.338, -0.65}
};

void MainTask::matchStep()
{
	auto& prop = Robot::instance().propulsion();
	switch(g_foo)
	{
	case 0:
		prop.executeTrajectory(allpoints, 2, 0.5,1,1);
		g_foo = 1;
		break;
	case 1:
		prop.executePointTo(allpoints[2], 1,1,1);
		g_foo = 2;
		break;
	case 2:
		prop.executeTrajectory(allpoints+1, 2, 0.5,1,1);
		g_foo = 3;
		break;
	default:
		break;
	}


	while(prop.state() != PropulsionController::State::Stopped)
	{
		delayTicks(1);
	}
}
void MainTask::taskFunction()
{
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
				if(remainingMatchTime() < 85)
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
