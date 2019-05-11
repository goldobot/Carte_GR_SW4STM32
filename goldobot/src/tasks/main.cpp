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

unsigned char g_temp_buffer[32];

MainTask::MainTask() :
	m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer))
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


struct MsgMatchStateChange
{
	MatchState match_state;
	Side side;
};

void MainTask::taskFunction()
{
	Robot::instance().mainExchangeIn().subscribe({40,50,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({90,90,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({166,166,&m_message_queue});

	MsgMatchStateChange msg{Robot::instance().matchState(), Robot::instance().side()};
	Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));
	Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&msg, sizeof(msg));

	while(1)
	{
		MsgMatchStateChange prev_state{Robot::instance().matchState(), Robot::instance().side()};

		while(m_message_queue.message_ready())
		{
			process_message();
		}
		auto match_state = Robot::instance().matchState();
		switch(match_state)
		{
		case MatchState::Idle:
			if( Hal::get_gpio(4))
			{
				Robot::instance().setSide(Side::Purple);
			} else
			{
				Robot::instance().setSide(Side::Yellow);
			}
			if(Hal::get_gpio(1))
			{
				Robot::instance().setMatchState(MatchState::PreMatch);

				if(Robot::instance().side() == Side::Yellow)
				{
					m_sequence_engine.startSequence(0);
				}
				if(Robot::instance().side() == Side::Purple)
				{
					m_sequence_engine.startSequence(1);
				}
			}
			break;
		case MatchState::PreMatch:
			{
				if(m_sequence_engine.state() == SequenceState::Idle)
				{
					Robot::instance().setMatchState(MatchState::WaitForStartOfMatch);
				}
			}
			break;
		case MatchState::WaitForStartOfMatch:
			if(!Hal::get_gpio(1))
			{
				Robot::instance().setStartMatchTime(xTaskGetTickCount());
				Robot::instance().setMatchState(MatchState::Match);

				if(Robot::instance().side() == Side::Yellow)
				{
					m_sequence_engine.startSequence(2);
				}
				if(Robot::instance().side() == Side::Purple)
				{
					m_sequence_engine.startSequence(3);
				}
			}
			break;

		case MatchState::Match:
			{
				if(remainingMatchTime() == 0)
				{
					Hal::set_gpio(0, false);
					Robot::instance().setMatchState(MatchState::PostMatch);
					Hal::set_motors_enable(false);
				}
			}
			break;
		case MatchState::Unconfigured:
			break;
		case MatchState::Debug:
			break;
		default:
			break;
		}
		m_sequence_engine.doStep();

		MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};
		if(post_state.match_state != prev_state.match_state &&
		   post_state.side == prev_state.side)
		{
			Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&post_state, sizeof(post_state));
			Robot::instance().mainExchangeOut().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&post_state, sizeof(post_state));
		}

		vTaskDelay(1);
	}
}

void MainTask::process_message()
{
	int msg_size = m_message_queue.message_size();
	switch(m_message_queue.message_type())
	{
	case CommMessageType::SetMatchState:
	{
		MatchState state;
		m_message_queue.pop_message((unsigned char*)&state, sizeof(MatchState));
		Robot::instance().setMatchState(state);
	}
	break;
	case CommMessageType::MainSequenceBeginLoad:
		m_sequence_engine.beginLoad();
		m_message_queue.pop_message(nullptr, 0);
		break;

	case CommMessageType::MainSequenceEndLoad:
		m_sequence_engine.endLoad();
		m_message_queue.pop_message(nullptr, 0);
		break;

	case CommMessageType::MainSequenceLoadData:
		m_message_queue.pop_message(g_temp_buffer, 32);
		m_sequence_engine.loadData(g_temp_buffer, msg_size);
		break;
	case CommMessageType::MainSequenceStartSequence:
		uint16_t seq_id;
			m_message_queue.pop_message((unsigned char*) (&seq_id),2);
			m_sequence_engine.startSequence(seq_id);
			break;
	case CommMessageType::PropulsionStateChanged:
		{
		uint8_t buff[2];
		m_message_queue.pop_message((unsigned char*) (&buff),2);
		if(buff[0] == 1)
		{
			m_sequence_engine.finishedMovement();
		}
		}
		break;
	case CommMessageType::ArmsStateChange:
		{
		uint8_t buff[2];
		m_message_queue.pop_message((unsigned char*) (&buff),2);
		if(buff[1] == 1)
		{
			m_sequence_engine.finishedArmMovement();
		}
		}
		break;

	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}

}


