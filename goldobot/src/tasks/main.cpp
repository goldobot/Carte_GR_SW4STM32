#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

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

void MainTask::matchStep()
{

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
				comm.send_message((uint16_t)CommMessageType::StartOfMatch,(char*)&clock,sizeof(clock));
				m_match_state = State::Match;
				m_start_of_match_time = clock;
				Hal::set_gpio(0, true);
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
