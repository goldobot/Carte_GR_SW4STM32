/*
 * fpga.cpp
 *
 *  Created on: 27 mai 2018
 *      Author: Grégoire
 */

#include "goldobot/tasks/fpga.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/platform/message_exchange.hpp"
#include "goldobot/robot.hpp"

#include <cstring>

#define SPI_FRAME_SZ 6
#define POOL_MAX_CNT 100000

using namespace goldobot;

FpgaTask::FpgaTask() : m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)) {
  generate_crc_table();
}

const char *FpgaTask::name() const { return "fpga"; }

void FpgaTask::taskFunction() {
  Robot::instance().mainExchangeIn().subscribe({30, 49, &m_message_queue});

  m_last_timestamp = hal::get_tick_count();

  while (1) {
    while (m_message_queue.message_ready()) {
      process_message();
    }

    if(m_cnt % 5 == 0)
    {


    // Read sensors
    unsigned int apb_data = 0;
    uint32_t apb_addr = 0x800084e4;  // gpio register
    if (goldo_fpga_master_spi_read_word(apb_addr, &apb_data) == 0) {
      if(apb_data != m_sensors_state)
      {
    	  m_sensors_state = apb_data;
          Robot::instance().exchangeInternal().pushMessage(CommMessageType::FpgaGpioState,
                                                                 (unsigned char *)&m_sensors_state, 4);
      }
    }
    }

    m_cnt++;
    if(m_cnt == 100)
    {
    	uint8_t watchdog_id = 2;
    	Robot::instance().exchangeInternal().pushMessage(CommMessageType::WatchdogReset,&watchdog_id, 1);

    	m_cnt = 0;
    }
    delay_periodic(1);
  } /* while(1) */
}

int FpgaTask::goldo_fpga_send_spi_frame(void) {
  hal::spi_read_write(4, spi_buf_in, spi_buf_out, SPI_FRAME_SZ);
  m_total_spi_frame_cnt++;
  return 0;
}

FpgaSpiTransactionStatus FpgaTask::spiTransaction(uint8_t command, FpgaSpiTransactionDir dir,
                                                  uint32_t arg, uint32_t &result) {
  spi_buf_out[0] = command;
  spi_buf_out[1] = (arg >> 24) & 0xff;
  spi_buf_out[2] = (arg >> 16) & 0xff;
  spi_buf_out[3] = (arg >> 8) & 0xff;
  spi_buf_out[4] = (arg)&0xff;
  spi_buf_out[5] = 0;

  std::memset(spi_buf_in, 0, sizeof(spi_buf_in));

  goldo_fpga_send_spi_frame();
  auto status = (dir == FpgaSpiTransactionDir::Write)
                    ? goldo_fpga_check_crc(spi_buf_out, spi_buf_in[5])
                    : goldo_fpga_check_crc(spi_buf_in, spi_buf_in[5]);
  if (status == FpgaSpiTransactionStatus::Ok) {
    result = (spi_buf_in[1] << 24) + (spi_buf_in[2] << 16) + (spi_buf_in[3] << 8) + (spi_buf_in[4]);
  }
  return status;
}

FpgaSpiTransactionStatus FpgaTask::spiTransaction(uint8_t command, FpgaSpiTransactionDir dir,
                                                  uint32_t arg, uint32_t &result, int retries) {
  FpgaSpiTransactionStatus status;
  for (int i = 0; i < retries; i++) {
    status = spiTransaction(command, dir, arg, result);
    if (status == FpgaSpiTransactionStatus::Ok) {
      return status;
    }
  }
  return status;
}

int FpgaTask::goldo_fpga_master_spi_read_word(unsigned int apb_addr, unsigned int *pdata) {
  uint32_t val;

  /* 1) sending APB_ADDR */
  if (spiTransaction(0x30, FpgaSpiTransactionDir::Write, apb_addr, val, 4) !=
      FpgaSpiTransactionStatus::Ok) {
    return 0;
  }

  /* 2) reading data */
  if (spiTransaction(0x50, FpgaSpiTransactionDir::Read, 0, val, 4) !=
      FpgaSpiTransactionStatus::Ok) {
    return 0;
  }
  *pdata = val;
  return 0;
}

int FpgaTask::goldo_fpga_master_spi_write_word(unsigned int apb_addr, unsigned int data) {
  uint32_t val;

  /* 1) sending APB_ADDR */
  if (spiTransaction(0x30, FpgaSpiTransactionDir::Write, apb_addr, val, 4) !=
      FpgaSpiTransactionStatus::Ok) {
    return 0;
  }

  /* 2) writing data */
  if (spiTransaction(0x40, FpgaSpiTransactionDir::Write, data, val, 4) !=
      FpgaSpiTransactionStatus::Ok) {
    return 0;
  }
  return 0;
}

unsigned int FpgaTask::goldo_fpga_get_version(void) {
  int result;
  unsigned int version = 0;
  unsigned int apb_addr = 0x8000800c;

  result = goldo_fpga_master_spi_read_word(apb_addr, &version);
  if (result != 0) {
    return 0xffffffff;
  }

  return version;
}

unsigned int goldo_fpga_servo_addr[] = {
    0x80008404, 0x8000840c, 0x80008414, 0x8000841c, 0x80008424, 0x8000842c,
    0x80008434, 0x8000843c, 0x80008444, 0x8000844c, 0x80008454, 0x8000845c,
};

/* robot 2018 :
   servo_id = [0..11]
   new_pw  = [0..0x40000]
*/
int FpgaTask::goldo_fpga_cmd_servo(int servo_id, unsigned int new_pos) {
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((servo_id < 0) || (servo_id > 11) || (new_pos > 0x00040000)) {
    return -1;
  }

  apb_addr = goldo_fpga_servo_addr[servo_id];

  result = goldo_fpga_master_spi_write_word(apb_addr, new_pos);
  if (result != 0) {
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
int FpgaTask::goldo_fpga_cmd_motor(int motor_id, int new_val) {
  int result;
  unsigned int apb_addr = 0x80008008;

  if ((motor_id < 0) || (motor_id > 2) || (new_val < -512) || ((new_val > 512))) {
    return -1;
  }

  apb_addr = goldo_fpga_motor_addr[motor_id];

  result = goldo_fpga_master_spi_write_word(apb_addr, new_val);
  if (result != 0) {
    return result;
  }

  return 0;
}

void FpgaTask::process_message() {
  auto message_type = (CommMessageType)m_message_queue.message_type();
  // auto message_size = m_message_queue.message_size();

  switch (message_type) {
    case CommMessageType::FpgaReadReg: {
      unsigned int apb_data = 0xdeadbeef;
      unsigned char buff[8];
      m_message_queue.pop_message(buff, 4);
      uint32_t apb_addr = *(uint32_t *)(buff);
      goldo_fpga_master_spi_read_word(apb_addr, &apb_data);
      std::memcpy(buff + 4, (unsigned char *)&apb_data, 4);
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::FpgaReadRegStatus,
                                                      (unsigned char *)buff, 8);
    } break;

    case CommMessageType::FpgaWriteReg: {
      unsigned char buff[8];
      m_message_queue.pop_message(buff, 8);
      uint32_t apb_addr = *(uint32_t *)(buff);
      uint32_t apb_data = *(uint32_t *)(buff + 4);
      goldo_fpga_master_spi_write_word(apb_addr, apb_data);
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  };
}

void FpgaTask::generate_crc_table() {
  int x, y, j;
  int idx;
  unsigned char crc_out, data_in;

  for (x = 0; x < 32; x++) {
    for (y = 0; y < 8; y++) {
      idx = x * 8 + y;
      data_in = swap_bits(idx);
      crc_out = 0;
      for (j = 0; j < 8; j++) {
        crc_out = MyLittleCRC(crc_out, ((data_in >> (7 - j)) & 1));
      }
      m_crc_table[idx] = crc_out;
    }
  }
}

unsigned char FpgaTask::swap_bits(unsigned char IN) {
  unsigned char x;

  x = IN;
  x = (x << 4) | (x >> 4);
  x = ((x & 0x33) << 2) | ((x & 0xcc) >> 2);
  x = ((x & 0x55) << 1) | ((x & 0xaa) >> 1);
  return x;
}

unsigned char FpgaTask::MyLittleCRC(unsigned char INCRC, unsigned char INBIT) {
  unsigned char x;
  unsigned char LFSR;

  x = INCRC;
  x = (x << 4) | (x >> 4);
  x = ((x & 0x33) << 2) | ((x & 0xcc) >> 2);
  x = ((x & 0x55) << 1) | ((x & 0xaa) >> 1);
  LFSR = x;

  LFSR = ((LFSR << 1) & 0xf8) | ((INBIT ^ (LFSR >> 7) ^ ((LFSR >> 1) & 0x01)) << 2) |
         ((INBIT ^ (LFSR >> 7) ^ ((LFSR)&0x01)) << 1) | (INBIT ^ (LFSR >> 7));

  x = LFSR;
  x = (x << 4) | (x >> 4);
  x = ((x & 0x33) << 2) | ((x & 0xcc) >> 2);
  x = ((x & 0x55) << 1) | ((x & 0xaa) >> 1);
  return x;
}

unsigned char FpgaTask::CalculateCRC(unsigned char INCRC, unsigned char INBYTE) {
  unsigned char x = INBYTE;
  x = (x << 4) | (x >> 4);
  x = ((x & 0x33) << 2) | ((x & 0xcc) >> 2);
  x = ((x & 0x55) << 1) | ((x & 0xaa) >> 1);
  return m_crc_table[INCRC ^ x];
}

FpgaSpiTransactionStatus FpgaTask::goldo_fpga_check_crc(unsigned char *buf5,
                                                        unsigned char recv_crc) {
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

  if (calc_crc_inv == 0x00) return FpgaSpiTransactionStatus::ApbBusy; /* apb busy */
  if (calc_crc != 0x00) return FpgaSpiTransactionStatus::CrcError;    /* spi bus glitch */
  return FpgaSpiTransactionStatus::Ok;
}
