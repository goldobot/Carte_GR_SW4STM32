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

#if 1 /* FIXME : DEBUG */
bool g_goldo_megakill_switch = false;
namespace goldobot {
  bool g_goldo_debug6 = false;
  bool g_goldo_debug7 = false;
  int g_dbg_goldo_vec[4] = {0};
};
#endif

void HeartbeatTask::taskFunction()
{
  int sync_period = 10;
  int sync_counter = 0;

#if 1 /* FIXME : DEBUG */
  g_goldo_megakill_switch = false;
  goldobot::g_goldo_debug6 = true;
  goldobot::g_goldo_debug7 = true;
  for (int i=0; i<4; i++) goldobot::g_dbg_goldo_vec[i] = 0;
#endif

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

#if 1 /* FIXME : DEBUG */
    if (sync_counter==1)
    {
      g_dbg_goldo_vec[0] = clock;
      RobotPose my_pose = Robot::instance().odometry().pose();
      double my_x_mm = 1000.0*my_pose.position.x;
      double my_y_mm = 1000.0*my_pose.position.y;
      double my_theta_deg = my_pose.yaw/M_PI*180.0*1000.0;
      g_dbg_goldo_vec[1] = my_x_mm;
      g_dbg_goldo_vec[2] = my_y_mm;
      g_dbg_goldo_vec[3] = my_theta_deg;
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DebugGoldoVect,
        (unsigned char*)&g_dbg_goldo_vec, sizeof(g_dbg_goldo_vec));
    }
#endif

#if 0 /* FIXME : DEBUG */
    g_goldo_megakill_switch = (Hal::get_gpio(2)!=0);
    if (g_goldo_megakill_switch) {
      Hal::disable_motors_pwm();
    }
#endif

    delay_periodic(100);
  }

}
