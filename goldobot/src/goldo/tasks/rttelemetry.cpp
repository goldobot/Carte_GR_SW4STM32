#include <cstring>

#include "hal/generic/hal.hpp"
#include "goldo/core/geometry.hpp"
#include "goldo/tasks/rttelemetry.hpp"
#include "goldo/robot.hpp"

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

bool g_goldo_megakill_switch = false;
bool g_goldo_debug6 = false;
bool g_goldo_debug7 = false;

bool debug_traj_flag = false;
int debug_num_points = 0;
short debug_traj_x_mm[16];
short debug_traj_y_mm[16];


#if 1 /* FIXME : TODO : refactor/improve */
#define RT_RCV_IDLE         0
#define RT_RCV_RECEIVING    1
#define RT_RCV_WAIT_PROCESS 2
unsigned int rt_rcv_state = RT_RCV_IDLE;
#define RT_RCV_BUFF_SZ 256
unsigned char rt_rcv_buff[RT_RCV_BUFF_SZ];

void rt_telemetry_cb(void)
{
  rt_rcv_state = RT_RCV_WAIT_PROCESS;
}
#endif


void RtTelemetryTask::taskFunction()
{
  unsigned int my_ticks = 0;
  int n_char = 0;
  unsigned char odo_send_buf[64];
  debug_traj_flag = false;
  rt_rcv_state = RT_RCV_IDLE;

  g_goldo_megakill_switch = false;
  g_goldo_debug6 = true;
  g_goldo_debug7 = true;

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

    uint32_t gpio = 0;
    for(int i=0; i<6; i++)
    {
      if(Hal::get_gpio(i)) gpio |= (1 << i);
    }
    PropulsionController::State prop_state= Robot::instance().propulsionState();
    if(prop_state!=PropulsionController::State::Stopped)
    {
      gpio |= (1 << 16);
    }
    if(prop_state==PropulsionController::State::Error)
    {
      gpio |= (1 << 17);
    }
    pw = (unsigned int *) &gpio;
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
    }

#if 1 /* FIXME : TODO : refactor/improve */
    switch (rt_rcv_state) {
    case RT_RCV_IDLE:
      std::memset(rt_rcv_buff,0,RT_RCV_BUFF_SZ);
      Hal::uart_receive_dma(2, (const char *)rt_rcv_buff, RT_RCV_BUFF_SZ);
      rt_rcv_state = RT_RCV_RECEIVING;
      break;
    case RT_RCV_RECEIVING:
      /* receive completion is detected in rt_telemetry_cb() */
      break;
    case RT_RCV_WAIT_PROCESS:
      /* Cmd msg : [x55 x24 x00 <frame_len> <seq_cnt> <msg_type> <msg_pload>] */
      /*  - <frame_len> : 1 byte , includes header */
      /*  - <seq_cnt>   : 4 bytes, little endian   */
      /*  - <msg_type>  : 2 bytes, little endian   */
      if ((rt_rcv_buff[0]==0x55) && 
          (rt_rcv_buff[1]==0x24) &&
          (rt_rcv_buff[2]==0x00) &&
          (rt_rcv_buff[3]> 0x0a)) {
        int frame_len = rt_rcv_buff[3];
        uint16_t msg_type = *((uint16_t *)((uint8_t *)(&rt_rcv_buff[8])));
        size_t msg_size = frame_len - 8 - 2;
        uint8_t *msg_payload = &rt_rcv_buff[10];
        Robot::instance().mainExchangeIn().pushMessage((CommMessageType)msg_type, msg_payload, msg_size);
      }
      rt_rcv_state = RT_RCV_IDLE;
      break;
    default:
      rt_rcv_state = RT_RCV_IDLE;
    }
#endif

#if 0
    /* FIXME : TODO : implement a remote "kill switch"? */
    g_goldo_megakill_switch = (Hal::get_gpio(2)!=0);
    if (g_goldo_megakill_switch) {
      Hal::disable_motors_pwm();
    }
#endif

    delay_periodic(50);
  }
}
