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
	unsigned int my_ticks = 0;

	while(1)
	{
		auto& comm = Robot::instance().comm();
		auto& main_task = Robot::instance().mainTask();
		if ((my_ticks%10)==0) {
			uint32_t clock = xTaskGetTickCount();
			comm.send_message(CommMessageType::Sync,"goldobot",8);
			comm.send_message(CommMessageType::Heartbeat,(char*)&clock,sizeof(clock));
		}
		{
			uint32_t gpio_mask = 
			  ((main_task.get_sequence_active_flag()?1:0)<<31) |
			  ((main_task.get_active_state_code()&7)<<24) |
			  (Hal::get_gpio(4)<<1) | 
			  (Hal::get_gpio(1));
			comm.send_message(CommMessageType::DbgGPIO,(char*)&gpio_mask,sizeof(gpio_mask));
		}
		delay_periodic(100);
		my_ticks++;
	}
}
