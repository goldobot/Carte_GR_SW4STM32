#include "hal/generic/hal.hpp"
#include "goldo/goldobot_main.h"
#include "goldo/robot.hpp"
#include "goldo/tasks/uart_comm.hpp"


goldobot::UARTCommTask s_uart_comm_task;

void goldobot_main()
{
  goldobot::Hal::init();
  goldobot::Robot::instance().init();
}
