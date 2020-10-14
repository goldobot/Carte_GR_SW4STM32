#include "goldobot/tasks/heartbeat.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/messages.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"

using namespace goldobot;

HeartbeatTask::HeartbeatTask() {}

const char* HeartbeatTask::name() const { return "heartbeat"; }

#if 1 /* FIXME : DEBUG */
bool g_goldo_megakill_switch = false;
namespace goldobot {
bool g_goldo_debug6 = false;
bool g_goldo_debug7 = false;
unsigned int g_dbg_goldo = 0;
};  // namespace goldobot
#endif

void HeartbeatTask::taskFunction() {
  int i;

  while (1) {
    uint32_t clock = hal::get_tick_count();
    auto& exchange = Robot::instance().mainExchangeOut();
    exchange.pushMessage(CommMessageType::Heartbeat, (unsigned char*)&clock, sizeof(clock));

    uint16_t remaining_time = Robot::instance().remainingMatchTime();
    exchange.pushMessage(CommMessageType::MatchRemainingTime, (unsigned char*)&remaining_time, 2);

    i++;
    if (i == 10) {
      i = 0;
      HeapStats_t heap_stats;
      vPortGetHeapStats(&heap_stats);
      exchange.pushMessage(CommMessageType::HeapStats, (unsigned char*)&heap_stats,
                           sizeof(heap_stats));
    }

    // gpio debug
    uint32_t gpio = 0;
    for (int i = 0; i < 6; i++) {
      if (hal::gpio_get(i)) gpio |= (1 << i);
    }

    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::GPIODebug,
                                                    (unsigned char*)&gpio, sizeof(gpio));

    messages::MsgMatchStateChange post_state{Robot::instance().matchState(),
                                             Robot::instance().side()};
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::MatchStateChange,
                                                   (unsigned char*)&post_state, sizeof(post_state));

    delay_periodic(100);
  }
}
