#include "goldobot/tasks/heartbeat.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/messages.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"

using namespace goldobot;

HeartbeatTask::HeartbeatTask() : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer))
		{}

const char* HeartbeatTask::name() const { return "heartbeat"; }

void HeartbeatTask::taskFunction() {
	Robot::instance().exchangeInternal().subscribe({34, 34, &m_message_queue});
	Robot::instance().exchangeInternal().subscribe({250, 250, &m_message_queue});

  int i;

  uint32_t watchdog_timestamps[8];

  for(int j = 0; j < 10; j++)
  {
	  uint32_t clock{0};
	  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::Heartbeat, (unsigned char*)&clock, sizeof(clock));
	  delay_periodic(10);
  }

  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::Reset, nullptr, 0);
  while (1) {
    uint32_t clock = hal::get_tick_count();

    auto& exchange = Robot::instance().mainExchangeOut();
    exchange.pushMessage(CommMessageType::Heartbeat, (unsigned char*)&clock, sizeof(clock));

    while(m_message_queue.message_ready())
    {
    	auto message_type = m_message_queue.message_type();
    	if(message_type == CommMessageType::FpgaGpioState)
    	{
    		uint32_t apb_data{0};
    		m_message_queue.pop_message((unsigned char*)&apb_data, sizeof(apb_data));
    		if(apb_data != m_fpga_gpio_state)
    		{
    			m_fpga_gpio_state_changed = true;
    			m_fpga_gpio_state = apb_data;
    		}
    	}else if(message_type == CommMessageType::WatchdogReset )
    	{
    		uint8_t watchdog_id{0};
    		m_message_queue.pop_message(&watchdog_id, 1);
    		if(watchdog_id < 8)
    		{
    			watchdog_timestamps[watchdog_id] = clock;
    		}
    	} else
    	{
    		m_message_queue.pop_message(nullptr, 0);
    	}
    }

    if (i == 0) {
      HeapStats_t heap_stats;
      vPortGetHeapStats(&heap_stats);
      exchange.pushMessage(CommMessageType::HeapStats, (unsigned char*)&heap_stats,
                           sizeof(heap_stats));
      uint8_t watchdogs[8];
      for(int k = 0; k < 8; k++)
      {
    	  watchdogs[k] = (watchdog_timestamps[k] + 1000 < clock);
    	  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::WatchdogStatus, watchdogs, 8);
      }
    }

   	checkGpioState();

    i++;
    if(i == 100)
    {
    	i = 0;
    }
    delay_periodic(10);
  }
}

void HeartbeatTask::checkGpioState()
{
  uint32_t gpio_state{0};
  for(int i = 0; i < 32; i++)
  {
	if(hal::gpio_get(i))
	{
		gpio_state |= 1 << i;
	}
  }

  bool has_changed = (gpio_state != m_gpio_state);
  if(has_changed || m_fpga_gpio_state_changed)
  {
	  m_fpga_gpio_state_changed = 0;
	  m_gpio_state = gpio_state;

     Robot::instance().mainExchangeOut().pushMessage(CommMessageType::SensorsState,
	 												(unsigned char *)&m_gpio_state, 8);
     Robot::instance().exchangeInternal().pushMessage(CommMessageType::SensorsState,
														  (unsigned char *)&m_gpio_state, 8);
  }

}
