#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

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
		auto& comm = Robot::instance().comm();
		comm.send_message(0,"goldobot",8);
		delayTicks(1000);
	}
}
