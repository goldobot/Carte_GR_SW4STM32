#include "goldobot/goldobot_main.h"

#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/uart_comm.hpp"

goldobot::UARTCommTask s_uart_comm_task;

void goldobot_main() {
  goldobot::hal::init();
  goldobot::Robot::instance().init();
}
