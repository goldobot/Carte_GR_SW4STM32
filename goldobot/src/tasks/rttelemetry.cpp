#include "goldobot/core/geometry.hpp"
#include "goldobot/tasks/rttelemetry.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace goldobot;

RtTelemetryTask::RtTelemetryTask()
{
}

const char* RtTelemetryTask::name() const
{
  return "rttelemetry";
}

bool debug_traj_flag = false;
int debug_num_points = 0;
short debug_traj_x_mm[16];
short debug_traj_y_mm[16];

void RtTelemetryTask::taskFunction()
{
  unsigned int my_ticks = 0;
  int n_char = 0;
  unsigned char odo_send_buf[64];
  debug_traj_flag = false;

  while(1)
  {
    unsigned int *pw;
    unsigned short *psw;
    unsigned char *pc;
    RobotPose my_pose;
    double my_x_mm;
    double my_y_mm;
    double my_theta_deg;
    int tmp_i;
    short tmp_s;
    char tmp_c;
    uint32_t clock = xTaskGetTickCount();
    int i;

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

    delay_periodic(50);

    if (debug_traj_flag)
    {
      n_char=0;

      odo_send_buf[n_char++] = 0x0a;
      odo_send_buf[n_char++] = 0x35;
      odo_send_buf[n_char++] = 0x00;
      tmp_c = debug_num_points;
      odo_send_buf[n_char++] = tmp_c;

      for (i=0; i<debug_num_points; i++) {
        tmp_s = debug_traj_x_mm[i];
        psw = (unsigned short *) &tmp_s;
        pc = (unsigned char *) psw;
        odo_send_buf[n_char++] = *(pc++);
        odo_send_buf[n_char++] = *(pc++);

        tmp_s = debug_traj_y_mm[i];
        psw = (unsigned short *) &tmp_s;
        pc = (unsigned char *) psw;
        odo_send_buf[n_char++] = *(pc++);
        odo_send_buf[n_char++] = *(pc++);
      }

      Hal::uart_transmit_dma(2, (const char *) odo_send_buf, n_char);

      debug_traj_flag = false;

      delay_periodic(50);
    }
    else
    {
      delay_periodic(50);
    }

  }
}
