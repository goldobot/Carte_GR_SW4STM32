/*
 * fpga.cpp
 *
 *  Created on: 27 mai 2018
 *      Author: Grégoire
 */

#include "goldobot/tasks/fpga.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "FreeRTOS.h"

#define SPI_FRAME_SZ 6
#define POOL_MAX_CNT 100000

using namespace goldobot;

FpgaTask::FpgaTask() {
}

void FpgaTask::taskFunction()
{
	//dynamixels_reset_all();
}

int FpgaTask::goldo_fpga_send_spi_frame(void) {
	int i;
	volatile uint8_t *spi_dr = (uint8_t *) 0x4001300c;
	volatile uint32_t *spi_sr = (uint32_t *) 0x40013008;
	int pool_cnt = 0;

	spi_buf_in[0] = *spi_dr;
	for (i=0; i<SPI_FRAME_SZ; i++) {
	pool_cnt = 0;
	while (((*spi_sr)&2)==0) {
	  pool_cnt++;
	  if (pool_cnt>POOL_MAX_CNT) return -1;
	}
	*spi_dr = spi_buf_out[i];
	pool_cnt = 0;
	while (((*spi_sr)&1)==0){
	  pool_cnt++;
	  if (pool_cnt>POOL_MAX_CNT) return -1;
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

int FpgaTask::goldo_fpga_columns_calib (void)
{
  // FIXME : TODO
  return 0;
}

int FpgaTask::goldo_fpga_columns_move (int col_id)
{
  // FIXME : TODO
  return 0;
}
