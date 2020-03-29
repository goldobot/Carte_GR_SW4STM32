#include "goldobot/tasks/main.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/messages.hpp"

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
	int match_duration = 98;
	return elapsed_time < match_duration ? match_duration - elapsed_time : 0;
}

void MainTask::taskFunction()
{
	Robot::instance().mainExchangeIn().subscribe({40,50,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({90,90,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({166,166,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({400,402,&m_message_queue});
	Robot::instance().mainExchangeIn().subscribe({322,322,&m_message_queue});

	messages::MsgMatchStateChange msg{Robot::instance().matchState(), Robot::instance().side()};

	// Config loop
	while(Robot::instance().matchState() == MatchState::Unconfigured)
	{
	while(m_message_queue.message_ready())
		{
			process_message_config();
		}
		vTaskDelay(1);
	}
	{
		messages::MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};

	}


	while(1)
	{
		messages::MsgMatchStateChange prev_state{Robot::instance().matchState(), Robot::instance().side()};

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
				m_start_of_match_time = xTaskGetTickCount();
				Robot::instance().setRemainingMatchTime(remainingMatchTime());
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
				Robot::instance().setRemainingMatchTime(remainingMatchTime());
				if(remainingMatchTime() == 0 || m_sequence_engine.state() == SequenceState::Idle)
				{
					Robot::instance().setMatchState(MatchState::Idle);
					m_sequence_engine.abortSequence();
					m_sequence_engine.startSequence(4);
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
		auto prev_seq_state = m_sequence_engine.state();
		m_sequence_engine.doStep();

		messages::MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};
		if(post_state.match_state != prev_state.match_state ||
		   post_state.side != prev_state.side)
		{	}

		vTaskDelay(1);
	}
}


void MainTask::process_message_config()
{
	int msg_size = m_message_queue.message_size();
	switch(m_message_queue.message_type())
	{
	case CommMessageType::RobotBeginLoadConfig:
	{
		m_message_queue.pop_message(nullptr, 0);
		Robot::instance().beginLoadConfig();
	}
	break;

	case CommMessageType::RobotLoadConfig:
	{
		m_message_queue.pop_message(m_scratchpad, msg_size);
		Robot::instance().loadConfig((char*)m_scratchpad, msg_size);
	}
	break;

	case CommMessageType::RobotEndLoadConfig:
	{
		uint16_t crc;
		m_message_queue.pop_message((unsigned char*)&crc, 2);
		uint8_t status = Robot::instance().endLoadConfig(crc);
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::RobotEndLoadConfigStatus, (unsigned char*)&status, 1);
	}
	break;
	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
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
	case CommMessageType::MainSequenceAbortSequence:
		m_message_queue.pop_message(nullptr,0);
		m_sequence_engine.abortSequence();
		break;
	case CommMessageType::PropulsionStateChanged:
		{
		uint8_t buff[2];
		m_message_queue.pop_message((unsigned char*) (&buff),2);
		m_sequence_engine.updatePropulsionState((PropulsionState)buff[0]);
		}
		break;
	case CommMessageType::ArmsStateChange:
		{
		uint8_t buff[2];
		m_message_queue.pop_message((unsigned char*) (&buff),2);
		m_sequence_engine.updateArmState((ArmState)buff[1]);
		}
		break;
	case CommMessageType::FpgaServoState:
		{
			uint8_t buff[2];
			m_message_queue.pop_message((unsigned char*) (&buff),2);
			m_sequence_engine.updateServoState(buff[0], buff[1]);
		}
		break;

	default:
		m_message_queue.pop_message(nullptr, 0);
		break;
	}

}


