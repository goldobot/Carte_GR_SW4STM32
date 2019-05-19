#include "goldobot/goldobot_main.h"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"


goldobot::UARTCommTask s_uart_comm_task;

void goldobot_main()
{
	goldobot::Hal::init();
	goldobot::Robot::instance().init();
}
