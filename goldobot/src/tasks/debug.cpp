#include "goldobot/tasks/debug.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/messages.hpp"
#include "goldobot/robot.hpp"

using namespace goldobot;

DebugTask::DebugTask() : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)) {}

const char* DebugTask::name() const { return "debug"; }

void DebugTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({20, 29, &m_message_queue});

  while (1) {
    doStep();
    delay(10);
  }
}

void DebugTask::doStep() {
  while (m_message_queue.message_ready()) {
    processMessage();
  }
}

void DebugTask::processMessage() {
  unsigned char buff[16];
  auto& exchange_out = Robot::instance().mainExchangeOut();

  auto message_type = (CommMessageType)m_message_queue.message_type();

  switch (message_type) {
    case CommMessageType::DbgGpioGet: {
      m_message_queue.pop_message(buff, 1);
      bool val = hal::gpio_get(buff[0]);
      buff[1] = val ? 1 : 0;
      exchange_out.pushMessage(CommMessageType::DbgGpioGet, buff, 2);
    } break;
    case CommMessageType::DbgGpioSet: {
      m_message_queue.pop_message(buff, 2);
      hal::gpio_set(buff[0], buff[1]);
    } break;
    case CommMessageType::DbgPwmSet: {
      m_message_queue.pop_message(buff, 5);
      float pwm = *reinterpret_cast<float*>(&buff[1]);
      hal::pwm_set(buff[0], pwm);
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}
