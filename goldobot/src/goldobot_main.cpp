#include "goldobot/goldobot_main.h"
#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/tasks/uart_comm.hpp"
#include "goldobot/hal.hpp"

goldobot::PropulsionTask s_propulsion_task;
goldobot::UARTCommTask s_uart_comm_task;

void goldobot_main()
{
	goldobot::Hal::init();
	//! \todo: select match or not
	s_propulsion_task.init();
	s_uart_comm_task.init();
}
