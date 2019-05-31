#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/messages.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

HeartbeatTask::HeartbeatTask()
{
}

const char* HeartbeatTask::name() const
{
	return "heartbeat";
}

void HeartbeatTask::taskFunction()
{
	while(1)
	{
		uint32_t clock = xTaskGetTickCount();
		auto& exchange = Robot::instance().mainExchangeOut();
		exchange.pushMessage(CommMessageType::Sync,(unsigned char*)"goldobot",8);
		exchange.pushMessage(CommMessageType::Heartbeat,(unsigned char*)&clock,sizeof(clock));

		uint16_t remaining_time = Robot::instance().remainingMatchTime();
		exchange.pushMessage(CommMessageType::MatchRemainingTime,(unsigned char*)&remaining_time, 2);


		//gpio debug
		uint32_t gpio = 0;
		for(int i=0; i<6; i++)
		{
			if(Hal::get_gpio(i)) gpio |= (1 << i);
		}
		Robot::instance().mainExchangeOut().pushMessage(
						CommMessageType::GPIODebug,
						(unsigned char*)&gpio, sizeof(gpio));

		messages::MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&post_state, sizeof(post_state));


		delay_periodic(100);
	}
}
