#include <math.h>

#include "goldobot/tasks/heartbeat.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

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

bool g_goldo_megakill_switch = false;
#if 1 /* FIXME : DEBUG : NUCLEO ESTIMATED POSITION VECTOR */
int n_char = 0;
unsigned char odo_send_buf[64];
unsigned int my_time_ms;
#endif

void HeartbeatTask::taskFunction()
{
	unsigned int my_ticks = 0;

	g_goldo_megakill_switch = false;

	while(1)
	{
#if 1 /* FIXME : DEBUG : NUCLEO ESTIMATED POSITION VECTOR */
		unsigned int *pw;
		unsigned short *psw;
		unsigned char *pc;
		RobotPose my_pose;
		double my_x_mm;
		double my_y_mm;
		double my_theta_deg;
		int tmp_i;
		short tmp_s;
		uint32_t clock = xTaskGetTickCount();

		n_char=0;

		my_pose = Robot::instance().odometry().pose();
		my_x_mm = 1000.0*my_pose.position.x;
		my_y_mm = 1000.0*my_pose.position.y;
		my_theta_deg = my_pose.yaw/M_PI*180.0*1000.0;

		odo_send_buf[n_char++] = 0x0a;
		odo_send_buf[n_char++] = 0x35;
		odo_send_buf[n_char++] = 0x0a;
		odo_send_buf[n_char++] = 0x35;

		//pw = (unsigned int *) &my_ticks;
		pw = (unsigned int *) &clock;
		pc = (unsigned char *) pw;
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);

		tmp_s = my_x_mm;
		psw = (unsigned short *) &tmp_s;
		pc = (unsigned char *) psw;
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);

		tmp_s = my_y_mm;
		psw = (unsigned short *) &tmp_s;
		pc = (unsigned char *) psw;
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);

		tmp_i = my_theta_deg;
		pw = (unsigned int *) &tmp_i;
		pc = (unsigned char *) pw;
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);
		odo_send_buf[n_char++] = *(pc++);

		//Hal::uart_transmit(2, (const char *) odo_send_buf, n_char, false);
		Hal::uart_transmit_dma(2, (const char *) odo_send_buf, n_char);
#endif

		auto& comm = Robot::instance().comm();
		auto& main_task = Robot::instance().mainTask();
		if ((my_ticks%10)==0) {
			//uint32_t clock = xTaskGetTickCount();
			comm.send_message(CommMessageType::Sync,"goldobot",8);
			comm.send_message(CommMessageType::Heartbeat,(char*)&clock,sizeof(clock));
		}
		{
			uint32_t gpio_mask = 
			  ((main_task.get_sequence_active_flag()?1:0)<<31) |
			  ((main_task.get_active_state_code()&7)<<24) |
			  (Hal::get_gpio(2)<<2) | 
			  (Hal::get_gpio(4)<<1) | 
			  (Hal::get_gpio(1));
			comm.send_message(CommMessageType::DbgGPIO,(char*)&gpio_mask,sizeof(gpio_mask));
		}

#if 1 /* FIXME : DEBUG */
        g_goldo_megakill_switch = (Hal::get_gpio(2)!=0);
        if (g_goldo_megakill_switch) {
            Hal::disable_motors_pwm();
        }
#endif

		delay_periodic(100);
		my_ticks++;
	}
}
