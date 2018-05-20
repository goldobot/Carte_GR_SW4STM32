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
		auto& comm = Robot::instance().comm();
		comm.send_message(0,"goldobot",8);
		comm.send_message((uint16_t)CommMessageType::Heartbeat,(char*)&clock,sizeof(clock));
		delay_periodic(1000);
	}
}
