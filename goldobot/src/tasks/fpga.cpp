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
#include "FreeRTOS.h"
#include "stm32f3xx_hal.h"
#include <cstring>

#define SPI_FRAME_SZ 6
#define POOL_MAX_CNT 100000

using namespace goldobot;

extern "C"
{
	extern SPI_HandleTypeDef hspi1;
}

FpgaTask::FpgaTask():
	m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)){
}

const char* FpgaTask::name() const
{
	return "fpga";
}
void FpgaTask::taskFunction()
{
	Robot::instance().mainExchangeIn().subscribe({256,322,&m_message_queue});

	while(1)
	{
		while(m_message_queue.message_ready())
		{
			process_message();
		}
		delay(1);

	}
}

int FpgaTask::goldo_fpga_send_spi_frame(void) {
	HAL_SPI_TransmitReceive_IT(&hspi1, spi_buf_out, spi_buf_in,SPI_FRAME_SZ);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
	{

	}
	return 0;
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
  if (result!=0) {
    return result;
  }

  /* 2) reading data */
  spi_buf_out[0] = 0x50;
  spi_buf_out[1] = 0;
  spi_buf_out[2] = 0;
  spi_buf_out[3] = 0;
  spi_buf_out[4] = 0;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  if (result!=0) {
    return result;
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
  if (result!=0) {
    return result;
  }

  /* 2) writing data */
  spi_buf_out[0] = 0x40;
  spi_buf_out[1] = (data>>24) & 0xff;
  spi_buf_out[2] = (data>>16) & 0xff;
  spi_buf_out[3] = (data>>8) & 0xff;
  spi_buf_out[4] = (data) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_fpga_send_spi_frame();
  if (result!=0) {
    return result;
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

unsigned int goldo_fpga_stp_addr[] = {
  0x800084c4,
  0x800084cc,
};

/* robot 2018 :
   stp_id = [0..1]
   new_pos  = [0..0x10000]
*/
int FpgaTask::goldo_fpga_cmd_stepper (int stp_id, unsigned int new_pos)
{
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((stp_id<0) || (stp_id>1) || (new_pos>0x00010000)) {
    return -1;
  }

  apb_addr = goldo_fpga_stp_addr[stp_id];

  result = goldo_fpga_master_spi_write_word (apb_addr, new_pos);
  if (result!=0) {
    return result;
  }

  return 0;
}

int FpgaTask::goldo_fpga_get_stepper_pos (int stp_id, unsigned int *new_pos)
{
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((stp_id<0) || (stp_id>1) || (new_pos==0)) {
    return -1;
  }

  apb_addr = goldo_fpga_stp_addr[stp_id];

  result = goldo_fpga_master_spi_read_word (apb_addr, new_pos);
  if (result!=0) {
    return result;
  }
  *new_pos = (*new_pos)>>16;

  return 0;
}

#define FPGA_COLUMNS_CALIB_TIMEOUT 8000 /* FIXME : TODO : tune */

int FpgaTask::goldo_fpga_columns_calib (void)
{
  int result;
  unsigned int apb_addr = 0x800084f0;

  /* calib right */
  result = goldo_fpga_master_spi_write_word (apb_addr, 11);
  if (result!=0) {
    return result;
  }

  delay_periodic(FPGA_COLUMNS_CALIB_TIMEOUT);

  goldo_fpga_cmd_stepper (0, 0x4000);

  delay_periodic(FPGA_COLUMNS_CALIB_TIMEOUT);

  /* calib left */
  result = goldo_fpga_master_spi_write_word (apb_addr, 13);
  if (result!=0) {
    return result;
  }

  delay_periodic(FPGA_COLUMNS_CALIB_TIMEOUT);

  goldo_fpga_columns_move (2);

  delay_periodic(FPGA_COLUMNS_CALIB_TIMEOUT);

  return 0;
}

int FpgaTask::goldo_fpga_columns_move (int col_id)
{
  int result;
  unsigned int apb_addr = 0x800084f0;

  if ((col_id<1) || (col_id>3)) {
    return -1;
  }

  result = goldo_fpga_master_spi_write_word (apb_addr, col_id);
  if (result!=0) {
    return result;
  }

  return 0;
}

int FpgaTask::goldo_fpga_set_columns_offset (int col_id, int col_offset)
{
  int result;

  if ((col_id<1) || (col_id>3)) {
    return -1;
  }

  if ((col_offset<-10000) || (col_id>10000)) {
    return -1;
  }

  /* FPGA BAL1 */
  result = goldo_fpga_master_spi_write_word (0x800084f4, col_offset);
  if (result!=0) {
    return result;
  }

  /* FPGA BAL0 */
  result = goldo_fpga_master_spi_write_word (0x800084f0, col_id+30);
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
			if(Robot::instance().fpgaTask().goldo_fpga_master_spi_read_word(apb_addr, &apb_data)!=0)
			{
				apb_data = 0xdeadbeef;
			}
			std::memcpy(buff+4, (unsigned char *)&apb_data, 4);
			Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaDbgReadReg, (unsigned char *)buff, 8);
		}
		break;
	case CommMessageType::FpgaDbgWriteReg:
		{
			unsigned char buff[8];
			m_message_queue.pop_message(buff, 8);
			uint32_t apb_addr = *(uint32_t*)(buff);
			uint32_t apb_data = *(uint32_t*)(buff+4);
			Robot::instance().fpgaTask().goldo_fpga_master_spi_write_word(apb_addr, apb_data);
		}
		break;
	default:
		break;
	};
}

