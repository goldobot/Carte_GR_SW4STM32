#include "hal/generic/hal.hpp"
#include "goldo/tasks/heartbeat.hpp"
#include "goldo/robot.hpp"
#include "goldo/messages.hpp"

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

#if 1 /* FIXME : DEBUG (a virer) */
extern bool g_goldo_debug6;
extern bool g_goldo_debug7;
#endif

void HeartbeatTask::taskFunction()
{
  int sync_period = 10;
  int sync_counter = 0;

  while(1)
  {
    uint32_t clock = xTaskGetTickCount();
    auto& exchange = Robot::instance().mainExchangeOut();
    if (sync_counter==0)
      exchange.pushMessage(CommMessageType::Sync,(unsigned char*)"goldobot",8);
    if (sync_counter==(sync_period-1))
      sync_counter=0;
    else
      sync_counter++;
    exchange.pushMessage(CommMessageType::Heartbeat,(unsigned char*)&clock,sizeof(clock));

    uint16_t remaining_time = Robot::instance().remainingMatchTime();
    exchange.pushMessage(CommMessageType::MatchRemainingTime,(unsigned char*)&remaining_time, 2);

    //gpio debug
    uint32_t gpio = 0;
    for(int i=0; i<6; i++)
    {
      if(Hal::get_gpio(i)) gpio |= (1 << i);
    }
#if 1 /* FIXME : DEBUG (a virer) */
    if(g_goldo_debug6)
    {
      gpio |= (1 << 6);
    }
    if(g_goldo_debug7)
    {
      gpio |= (1 << 7);
    }
#endif
    Robot::instance().mainExchangeOut().pushMessage(
      CommMessageType::GPIODebug,
      (unsigned char*)&gpio, sizeof(gpio));

    messages::MsgMatchStateChange post_state{Robot::instance().matchState(), Robot::instance().side()};
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange, (unsigned char*)&post_state, sizeof(post_state));

    delay_periodic(100);
  }

}
