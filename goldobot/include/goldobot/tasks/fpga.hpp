#pragma once
#include "goldobot/config.hpp"
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {

enum class FpgaSpiTransactionStatus {
  Ok,
  CrcError,
  ApbBusy,
};

enum class FpgaSpiTransactionDir { Read, Write };

class FpgaTask : public Task {
 public:
  FpgaTask();
  const char *name() const override;
  void taskFunction() override;

  FpgaSpiTransactionStatus spiTransaction(uint8_t command, FpgaSpiTransactionDir dir, uint32_t arg,
                                          uint32_t &result);
  FpgaSpiTransactionStatus spiTransaction(uint8_t command, FpgaSpiTransactionDir dir, uint32_t arg,
                                          uint32_t &result, int retries);

  int goldo_fpga_master_spi_read_word(unsigned int apb_addr, unsigned int *pdata);
  int goldo_fpga_master_spi_write_word(unsigned int apb_addr, unsigned int data);
  unsigned int goldo_fpga_get_version(void);
  int goldo_fpga_cmd_servo(int servo_id, unsigned int new_pos);  // range 0 0x40000
  int goldo_fpga_cmd_motor(int motor_id,
                           int new_val);  // val -511,511 0 pompe droite 1 pompe gauche 2 tapis
  int goldo_fpga_cmd_stepper(int stp_id, unsigned int new_pos);
  int goldo_fpga_get_stepper_pos(int stp_id, unsigned int *new_pos);
  int goldo_fpga_columns_calib(void);
  int goldo_fpga_columns_move(int col_id);
  int goldo_fpga_set_columns_offset(int col_id, int col_offset);

 private:
  unsigned char spi_buf_out[8];
  unsigned char spi_buf_in[8];
  int goldo_fpga_send_spi_frame(void);
  void process_message();
  void updateEmergencyStop();

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[256];

  uint32_t m_gpio_sensors_state{0};
  uint32_t m_sensors_state{0};
  uint32_t m_last_timestamp{0};
  uint32_t m_next_emergency_stop_update_ts{0};
  uint8_t m_cnt{0};

  unsigned char m_crc_table[256];
  void generate_crc_table();
  unsigned char MyLittleCRC(unsigned char INCRC, unsigned char INBIT);
  unsigned char swap_bits(unsigned char IN);
  unsigned char CalculateCRC(unsigned char INCRC, unsigned char INBYTE);
  FpgaSpiTransactionStatus goldo_fpga_check_crc(unsigned char *buf5, unsigned char recv_crc);

  int m_total_spi_frame_cnt{0};
};
}  // namespace goldobot
