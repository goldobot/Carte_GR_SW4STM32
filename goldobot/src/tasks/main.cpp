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


MainTask::MainTask()
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

struct MsgMatchStateChange
{
	MatchState match_state;
	Side side;
};

void MainTask::taskFunction()
{
	while(1)
	{
		auto match_state = Robot::instance().matchState();
		switch(match_state)
		{
		case MatchState::Idle:
			if(!Hal::get_gpio(1))
			{
				if( Hal::get_gpio(4))
				{
					Robot::instance().setSide(Side::Green);
				} else
				{
					Robot::instance().setSide(Side::Orange);
				}
				Hal::set_motors_enable(true);
				Robot::instance().propulsion().enable();
				Robot::instance().setMatchState(MatchState::PreMatch);
				MsgMatchStateChange msg{Robot::instance().matchState(), Robot::instance().side()};
				Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
				Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));

			}
			break;
		case MatchState::WaitForStartOfMatch:
			if(Hal::get_gpio(1))
			{
				Robot::instance().setStartMatchTime(xTaskGetTickCount());
				Robot::instance().setMatchState(MatchState::Match);
				MsgMatchStateChange msg{Robot::instance().matchState(), Robot::instance().side()};
				Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
				Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
			}
			break;

		case MatchState::Match:
			{
				if(remainingMatchTime() == 0)
				{
					Hal::set_gpio(0, false);
					Robot::instance().setMatchState(MatchState::PostMatch);
					MsgMatchStateChange msg{Robot::instance().matchState(), Robot::instance().side()};
					Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
					Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
					//comm.send_message(CommMessageType::EndOfMatch,(char*)&clock,sizeof(clock));
					Hal::set_motors_enable(false);
				}
			}
			break;
		default:
			break;
		}
		vTaskDelay(1);
	}
}


