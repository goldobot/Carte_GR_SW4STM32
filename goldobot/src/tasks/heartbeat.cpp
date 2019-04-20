#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

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
		delay_periodic(1000);
	}
}
