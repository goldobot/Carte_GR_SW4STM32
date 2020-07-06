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

#if 1 /* FIXME : DEBUG */
bool g_goldo_megakill_switch = false;
namespace goldobot {
	bool g_goldo_debug6 = false;
	bool g_goldo_debug7 = false;
    unsigned int g_dbg_goldo = 0;
};
#endif

void HeartbeatTask::taskFunction()
{
#if 1 /* FIXME : DEBUG */
	g_goldo_megakill_switch = false;
    goldobot::g_goldo_debug6 = true;
    goldobot::g_goldo_debug7 = true;
    goldobot::g_dbg_goldo = 0;
#endif

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
		if(goldobot::g_goldo_debug6)
		{
			gpio |= (1 << 6);
		}
		if(goldobot::g_goldo_debug7)
		{
			gpio |= (1 << 7);
		}
		Robot::instance().mainExchangeOut().pushMessage(
						CommMessageType::GPIODebug,
						(unsigned char*)&gpio, sizeof(gpio));

		messages::MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};
		Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&post_state, sizeof(post_state));


		//	(unsigned char*)&g_dbg_goldo, sizeof(g_dbg_goldo));

#if 0 /* FIXME : DEBUG */
		g_goldo_megakill_switch = (Hal::get_gpio(2)!=0);
		if (g_goldo_megakill_switch) {
			Hal::disable_motors_pwm();
		}
#endif

		delay_periodic(100);
	}
}
