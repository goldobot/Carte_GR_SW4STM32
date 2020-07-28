/*
 * fpga.cpp
 *
 *  Created on: 27 mai 2018
 *      Author: Grégoire
 */

#include "goldobot/tasks/fpga.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/core/message_exchange.hpp"
//#include "stm32f3xx_hal.h"
#include <cstring>

#define SPI_FRAME_SZ 6
#define POOL_MAX_CNT 100000

using namespace goldobot;

FpgaTask::FpgaTask():
  m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)){
  generate_crc_table();
  m_last_crc                = 0;
  m_total_spi_frame_cnt     = 0;
  m_strange_err_cnt         = 0;
  m_addr1_crc_err_cnt       = 0;
  m_addr2_crc_err_cnt       = 0;
  m_write1_crc_err_cnt      = 0;
  m_write2_crc_err_cnt      = 0;
  m_read1_crc_err_cnt       = 0;
  m_read2_crc_err_cnt       = 0;
  m_apb_err_cnt             = 0;
  m_last_dbg                = 0;
}

const char* FpgaTask::name() const
{
  return "fpga";
}

void FpgaTask::taskFunction()
{
  Robot::instance().mainExchangeIn().subscribe({256,322,&m_message_queue});
  m_servos_config = Robot::instance().servosConfig();

  for(unsigned i = 0; i < 16; i++)
  {
    m_servos_positions[i] = -1;
    m_servos_target_positions[i] = 0;
  }

  for(unsigned i=0; i<12; i++)
  {
    goldo_fpga_cmd_servo(i, 0);
  }

  m_last_timestamp = Hal::get_tick_count();

  while(1)
  {
    while(m_message_queue.message_ready())
    {
      process_message();
    }

    // Read sensors
    unsigned int apb_data = 0;
    uint32_t apb_addr = 0x800084e4; // gpio register
    if(goldo_fpga_master_spi_read_word(apb_addr, &apb_data)!=0)
    {
      apb_data = 0xdeadbeef;
      delay(1);
      continue;
    }

    /* Little workaround to filter out bad data due to "unknown" fpga bugs */
    if ((apb_data & 0xffffffc0) != 0x00000000) {
      m_last_dbg = apb_data;
      m_strange_err_cnt++;
      delay(1);
      continue;
    }

    if(apb_data != m_sensors_state)
    {
      m_sensors_state = apb_data;
      Robot::instance().setSensorsState(apb_data),
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::SensorsChange, (unsigned char *)&m_sensors_state, 4);
    }

    //Recompute servo targets
    uint32_t ts = Hal::get_tick_count();
    float delta_t = (ts - m_last_timestamp)*1e-3;
    m_last_timestamp = ts;
    for(int i=0; i < m_servos_config->num_servos; i++)
    {
      if(m_servos_positions[i] < 0)
      {
        continue;
      }

      bool was_moving=false;

      if(m_servos_positions[i] > m_servos_target_positions[i])
      {
        m_servos_positions[i] = std::max<float>(m_servos_target_positions[i], m_servos_positions[i] - m_servos_speeds[i] * delta_t);
        goldo_fpga_cmd_servo(m_servos_config->servos[i].id, (unsigned int)m_servos_positions[i]);
        was_moving =true;
      }

      if(m_servos_positions[i] < m_servos_target_positions[i])
      {
        m_servos_positions[i] = std::min<float>(m_servos_target_positions[i], m_servos_positions[i] + m_servos_speeds[i] * delta_t);
        goldo_fpga_cmd_servo(m_servos_config->servos[i].id, (unsigned int)m_servos_positions[i]);
        was_moving = true;
      }

      if(was_moving && m_servos_positions[i] == m_servos_target_positions[i])
      {
        unsigned char buff[2] = {(unsigned char)i, false};
        Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaServoState, (unsigned char *)buff, 2);
      }

    }

    delay(1);
  } /* while(1) */
}

int FpgaTask::goldo_fpga_send_spi_frame(void) {
    Hal::send_spi_frame(spi_buf_out, spi_buf_in);  
  m_total_spi_frame_cnt++;
  return 0;
#if 0 /* FIXME : TODO : cleanup (old code) */
/*
  int i;
  volatile uint8_t *spi_dr = (uint8_t *) 0x4001300c;
  volatile uint32_t *spi_sr = (uint32_t *) 0x40013008;
  int pool_cnt = 0;

  spi_buf_in[0] = *spi_dr;
  for (i=0; i<SPI_FRAME_SZ; i++) {
  pool_cnt = 0;
  while (((*spi_sr)&2)==0) {
    pool_cnt++;
    if (pool_cnt>POOL_MAX_CNT)
      return -1;
  }
  *spi_dr = spi_buf_out[i];
  pool_cnt = 0;
  while (((*spi_sr)&1)==0){
    pool_cnt++;
    if (pool_cnt>POOL_MAX_CNT)
      return -1;
  }
  spi_buf_in[i] = *spi_dr;
  }
  //while (((*spi_sr)&1)==0);
  spi_buf_in[SPI_FRAME_SZ] = *spi_dr;
  return 0;
*/
#endif
}

int FpgaTask::goldo_fpga_master_spi_read_word (unsigned int apb_addr, unsigned int *pdata)
{
  int i;
  uint32_t val;
  int result;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  m_last_crc = (spi_buf_in[0]<<24) | (spi_buf_in[5]<<16);
  if (result!=0) {
    return result;
  }

  result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
  if (result!=0) {
    m_addr1_crc_err_cnt++;
    result = goldo_fpga_send_spi_frame();
    m_last_crc = (spi_buf_in[0]<<24) | (spi_buf_in[5]<<16);
    if (result!=0) {
      return result;
    }

    result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
    if (result!=0) {
      m_addr2_crc_err_cnt++;
      return result;
    }
  }


  /* 2) reading data */
  spi_buf_out[0] = 0x50;
  spi_buf_out[1] = 0;
  spi_buf_out[2] = 0;
  spi_buf_out[3] = 0;
  spi_buf_out[4] = 0;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  m_last_crc |= (spi_buf_in[0]<<8) | (spi_buf_in[5]);
  if (result!=0) {
    return result;
  }

  result = goldo_fpga_check_crc(spi_buf_in, spi_buf_in[5]);
  if (result!=0) {
    if (result==-1) m_read1_crc_err_cnt++;
    if (result==-2) m_apb_err_cnt++;
    result = goldo_fpga_send_spi_frame();
    m_last_crc &= 0x0000ffff;
    m_last_crc |= (spi_buf_in[0]<<8) | (spi_buf_in[5]);
    if (result!=0) {
      return result;
    }

    result = goldo_fpga_check_crc(spi_buf_in, spi_buf_in[5]);
    if (result!=0) {
      if (result==-1) m_read2_crc_err_cnt++;
      if (result==-2) m_apb_err_cnt++;
      return result;
    }
  }


  val =
    (spi_buf_in[1]<<24) +
    (spi_buf_in[2]<<16) +
    (spi_buf_in[3]<<8)  +
    (spi_buf_in[4]);
  *pdata = val;

  return 0;
}


int FpgaTask::goldo_fpga_master_spi_write_word (unsigned int apb_addr, unsigned int data)
{
  int i;
  int result;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  m_last_crc = (spi_buf_in[0]<<24) | (spi_buf_in[5]<<16);
  if (result!=0) {
    return result;
  }

  result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
  if (result!=0) {
    m_addr1_crc_err_cnt++;
    result = goldo_fpga_send_spi_frame();
    m_last_crc = (spi_buf_in[0]<<24) | (spi_buf_in[5]<<16);
    if (result!=0) {
      return result;
    }

    result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
    if (result!=0) {
      m_addr2_crc_err_cnt++;
      return result;
    }
  }


  /* 2) writing data */
  spi_buf_out[0] = 0x40;
  spi_buf_out[1] = (data>>24) & 0xff;
  spi_buf_out[2] = (data>>16) & 0xff;
  spi_buf_out[3] = (data>>8) & 0xff;
  spi_buf_out[4] = (data) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  m_last_crc |= (spi_buf_in[0]<<8) | (spi_buf_in[5]);
  if (result!=0) {
    return result;
  }

  result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
  if (result!=0) {
    if (result==-1) m_write1_crc_err_cnt++;
    if (result==-2) m_apb_err_cnt++;
    result = goldo_fpga_send_spi_frame();
    m_last_crc = (spi_buf_in[0]<<24) | (spi_buf_in[5]<<16);
    if (result!=0) {
      return result;
    }

    result = goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5]);
    if (result!=0) {
      if (result==-1) m_write2_crc_err_cnt++;
      if (result==-2) m_apb_err_cnt++;
      return result;
    }
  }


  return 0;
}

unsigned int FpgaTask::goldo_fpga_get_version (void)
{
  int result;
  unsigned int version = 0;
  unsigned int apb_addr = 0x8000800c;

  result = goldo_fpga_master_spi_read_word (apb_addr, &version);
  if (result!=0) {
    return 0xffffffff;
  }

  return version;
}

unsigned int goldo_fpga_servo_addr[] = {
  0x80008404,
  0x8000840c,
  0x80008414,
  0x8000841c,
  0x80008424,
  0x8000842c,
  0x80008434,
  0x8000843c,
  0x80008444,
  0x8000844c,
  0x80008454,
  0x8000845c,
};

/* robot 2018 :
   servo_id = [0..11]
   new_pw  = [0..0x40000]
*/
int FpgaTask::goldo_fpga_cmd_servo (int servo_id, unsigned int new_pos)
{
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((servo_id<0) || (servo_id>11) || (new_pos>0x00040000)) {
    return -1;
  }

  apb_addr = goldo_fpga_servo_addr[servo_id];

  result = goldo_fpga_master_spi_write_word (apb_addr, new_pos);
  if (result!=0) {
    return result;
  }

  return 0;
}

unsigned int goldo_fpga_motor_addr[] = {
  0x80008484,
  0x8000848c,
  0x80008494,
};

/* robot 2018 :
   pompe droite : motor_id = 0
   pompe gauche : motor_id = 1
   moteur tapis : motor_id = 2
   new_pw  = [0..0x200]
*/
int FpgaTask::goldo_fpga_cmd_motor (int motor_id, int new_val)
{
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((motor_id<0) || (motor_id>2) ||
      (new_val<-512) || ((new_val>512))) {
    return -1;
  }

  apb_addr = goldo_fpga_motor_addr[motor_id];

  result = goldo_fpga_master_spi_write_word (apb_addr, new_val);
  if (result!=0) {
    return result;
  }

  return 0;
}

void FpgaTask::process_message()
{
  auto message_type = (CommMessageType)m_message_queue.message_type();
  //auto message_size = m_message_queue.message_size();

  switch(message_type)
  {
  case CommMessageType::FpgaDbgReadReg:
    {
      unsigned int apb_data = 0xdeadbeef;
      unsigned char buff[8];
      m_message_queue.pop_message(buff, 4);
      uint32_t apb_addr = *(uint32_t*)(buff);
      if(goldo_fpga_master_spi_read_word(apb_addr, &apb_data)!=0)
      {
        apb_data = 0xdeadbeef;
      }
      std::memcpy(buff+4, (unsigned char *)&apb_data, 4);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaDbgReadReg, (unsigned char *)buff, 8);
    }
    break;
  case CommMessageType::FpgaDbgReadRegCrc:
    {
      unsigned int apb_data = 0xdeadbeef;
      unsigned char buff[12];
      m_message_queue.pop_message(buff, 4);
      uint32_t apb_addr = *(uint32_t*)(buff);
      if(goldo_fpga_master_spi_read_word(apb_addr, &apb_data)!=0)
      {
        apb_data = 0xdeadbeef;
      }
      std::memcpy(buff+4, (unsigned char *)&apb_data, 4);
      std::memcpy(buff+8, (unsigned char *)&m_last_crc, 4);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaDbgReadRegCrc, (unsigned char *)buff, 12);
    }
    break;
  case CommMessageType::FpgaDbgWriteReg:
    {
      unsigned char buff[12];
      m_message_queue.pop_message(buff, 8);
      uint32_t apb_addr = *(uint32_t*)(buff);
      uint32_t apb_data = *(uint32_t*)(buff+4);
      goldo_fpga_master_spi_write_word(apb_addr, apb_data);
      std::memcpy(buff+8, (unsigned char *)&m_last_crc, 4);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaDbgReadRegCrc, (unsigned char *)buff, 12);
    }
    break;
  case CommMessageType::FpgaDbgGetErrCnt:
    {
#define ERR_CNT_SZ (16*4)
      unsigned char buff[ERR_CNT_SZ];
      std::memset(buff, 0, ERR_CNT_SZ);
      m_message_queue.pop_message(nullptr, 0);
      std::memcpy(buff+ 0, (unsigned char *)&m_total_spi_frame_cnt, 4);
      std::memcpy(buff+ 4, (unsigned char *)&m_strange_err_cnt    , 4);
      std::memcpy(buff+ 8, (unsigned char *)&m_addr1_crc_err_cnt  , 4);
      std::memcpy(buff+12, (unsigned char *)&m_addr2_crc_err_cnt  , 4);
      std::memcpy(buff+16, (unsigned char *)&m_write1_crc_err_cnt , 4);
      std::memcpy(buff+20, (unsigned char *)&m_write2_crc_err_cnt , 4);
      std::memcpy(buff+24, (unsigned char *)&m_read1_crc_err_cnt  , 4);
      std::memcpy(buff+28, (unsigned char *)&m_read2_crc_err_cnt  , 4);
      std::memcpy(buff+32, (unsigned char *)&m_apb_err_cnt        , 4);
      std::memcpy(buff+36, (unsigned char *)&m_last_dbg           , 4);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaDbgGetErrCnt, (unsigned char *)buff, ERR_CNT_SZ);
      goldo_send_log_uint32("m_total_spi_frame_cnt=%u", 
                            (uint32_t)m_total_spi_frame_cnt);
      goldo_send_log_uint32("m_strange_err_cnt=%u ", 
                            (uint32_t)m_strange_err_cnt);
      goldo_send_log_uint32("m_addr1_crc_err_cnt=%u ", 
                            (uint32_t)m_addr1_crc_err_cnt);
      goldo_send_log_uint32("m_addr2_crc_err_cnt=%u ", 
                            (uint32_t)m_addr2_crc_err_cnt);
      goldo_send_log_uint32("m_write1_crc_err_cnt=%u ", 
                            (uint32_t)m_write1_crc_err_cnt);
      goldo_send_log_uint32("m_write2_crc_err_cnt=%u ", 
                            (uint32_t)m_write2_crc_err_cnt);
      goldo_send_log_uint32("m_read1_crc_err_cnt=%u ", 
                            (uint32_t)m_read1_crc_err_cnt);
      goldo_send_log_uint32("m_read2_crc_err_cnt=%u ", 
                            (uint32_t)m_read2_crc_err_cnt);
      goldo_send_log_uint32("m_apb_err_cnt=%u ", 
                            (uint32_t)m_apb_err_cnt);
    }
    break;
  case CommMessageType::FpgaCmdDCMotor:
  {
    unsigned char buff[3];
    m_message_queue.pop_message(buff, 3);
    int motor_id = buff[0];
    int pwm = *(int16_t*)(buff+1);
    goldo_fpga_cmd_motor(motor_id, pwm);
  }
  break;
  case CommMessageType::FpgaCmdServo:
  {
    unsigned char buff[4];
    m_message_queue.pop_message(buff, 4);
    int motor_id = buff[0];
    int pwm = *(uint16_t*)(buff+1);
    if(m_servos_positions[motor_id] < 0)
    {
      m_servos_positions[motor_id] = pwm;
      goldo_fpga_cmd_servo(m_servos_config->servos[motor_id].id, pwm);
    }
    // Send message when servo start moving
    if(m_servos_target_positions[motor_id] != pwm)
    {
      unsigned char buff[2] = {(unsigned char)motor_id, true};
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaServoState, (unsigned char *)buff, 2);
    } else
    {
      unsigned char buff[2] = {(unsigned char)motor_id, false};
      Robot::instance().mainExchangeIn().pushMessage(CommMessageType::FpgaServoState, (unsigned char *)buff, 2);
    }
    m_servos_target_positions[motor_id] = pwm;
    m_servos_speeds[motor_id] = (m_servos_config->servos[motor_id].max_speed * buff[3])/100;
  }
  break;
  default:
    m_message_queue.pop_message(nullptr, 0);
    break;
  };
}

void FpgaTask::generate_crc_table()
{
  int x,y,j;
  int idx;
  unsigned char crc_out, data_in;

  for (x=0; x<32; x++) {
    for (y=0; y<8; y++) {
      idx = x*8+y;
      data_in = swap_bits(idx);
      crc_out = 0;
      for (j=0; j<8; j++) {
        crc_out = MyLittleCRC(crc_out, ((data_in>>(7-j))&1) );
      }
      m_crc_table[idx] = crc_out;
    }
  }
}

unsigned char FpgaTask::swap_bits(unsigned char IN)
{
  unsigned char x;

  x=IN;
  x = (x<<4) | (x>>4);
  x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
  x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
  return x;
}

unsigned char FpgaTask::MyLittleCRC(unsigned char INCRC, unsigned char INBIT)
{
  unsigned char x;
  unsigned char LFSR;

  x=INCRC;
  x = (x<<4) | (x>>4);
  x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
  x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
  LFSR = x;

  LFSR = 
    ((LFSR<<1)&0xf8) |
    ((INBIT ^ (LFSR>>7) ^ ((LFSR>>1)&0x01))<<2) |
    ((INBIT ^ (LFSR>>7) ^ ((LFSR)&0x01))<<1) |
    (INBIT ^ (LFSR>>7));

  x=LFSR;
  x = (x<<4) | (x>>4);
  x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
  x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
  return x;
}

unsigned char FpgaTask::CalculateCRC(unsigned char INCRC, unsigned char INBYTE)
{
  unsigned char x=INBYTE;
  x = (x<<4) | (x>>4);
  x = ((x & 0x33)<<2) | ((x & 0xcc)>>2);
  x = ((x & 0x55)<<1) | ((x & 0xaa)>>1);
  return m_crc_table[INCRC ^ x];
}

int FpgaTask::goldo_fpga_check_crc(unsigned char *buf5, unsigned char recv_crc)
{
  unsigned char calc_crc = 0;
  unsigned char calc_crc_inv = 0;

  calc_crc = CalculateCRC(calc_crc, buf5[0]);
  calc_crc = CalculateCRC(calc_crc, buf5[1]);
  calc_crc = CalculateCRC(calc_crc, buf5[2]);
  calc_crc = CalculateCRC(calc_crc, buf5[3]);
  calc_crc = CalculateCRC(calc_crc, buf5[4]);
  calc_crc_inv = calc_crc;
  calc_crc = CalculateCRC(calc_crc, recv_crc);

  calc_crc_inv = CalculateCRC(calc_crc_inv, ~recv_crc);

  if (calc_crc_inv==0x00) return -2; /* apb busy */
  if (calc_crc!=0x00) return -1; /* spi bus glitch */
  return 0;
}

void FpgaTask::goldo_send_log_uint32(const char *msg, unsigned int val)
{
  char dbg_log[64];
  std::memset(dbg_log, 0, 64);
  // \todo find a way to make this portable
  //sprintf(dbg_log, msg, val);
  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::NucleoLog, (unsigned char *)dbg_log, std::strlen(dbg_log));
}

