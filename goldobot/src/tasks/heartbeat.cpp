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
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DbgPrintf, (const unsigned char*)"goldorak", 8);
		/*
		uint32_t clock = xTaskGetTickCount();
		auto& exchange = Robot::instance().mainExchangeOut();
		exchange.pushMessage(CommMessageType::Heartbeat,(unsigned char*)&clock,sizeof(clock));



		//gpio debug
		uint32_t gpio = 0;
		for(int i=0; i<6; i++)
		{
			if(Hal::get_gpio(i)) gpio |= (1 << i);
		}
		Robot::instance().mainExchangeOut().pushMessage(
						CommMessageType::GPIO,
						(unsigned char*)&gpio, sizeof(gpio));*/


		delay_periodic(100);
	}
}
