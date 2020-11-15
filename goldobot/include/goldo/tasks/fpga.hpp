#pragma once
#include "goldo/tasks/task.hpp"
#include "goldo/core/message_queue.hpp"
#include "goldo/config.hpp"
#include <cstdint>

namespace goldobot
{
  struct FpgaServoState
  {
    uint16_t prev_value{0};
    uint16_t target_value{0};
    uint32_t prev_timestamp{0};
    uint32_t target_timestamp{0};
  };

  class FpgaTask : public Task
  {
  public:
    FpgaTask();
    const char* name() const override;
    void taskFunction() override;

    int goldo_fpga_master_spi_read_word (unsigned int apb_addr, unsigned int *pdata);
    int goldo_fpga_master_spi_write_word (unsigned int apb_addr, unsigned int data);
    unsigned int goldo_fpga_get_version (void);
    int goldo_fpga_cmd_servo (int servo_id, unsigned int new_pos);// range 0 0x40000
    int goldo_fpga_cmd_motor (int motor_id, int new_val);//val -511,511 0 pompe droite 1 pompe gauche 2 tapis
    int goldo_fpga_cmd_stepper (int stp_id, unsigned int new_pos);
    int goldo_fpga_get_stepper_pos (int stp_id, unsigned int *new_pos);
    int goldo_fpga_columns_calib (void);
    int goldo_fpga_columns_move (int col_id);
    int goldo_fpga_set_columns_offset (int col_id, int col_offset);

  private:
    unsigned char spi_buf_out[64];
    unsigned char spi_buf_in[64];
    unsigned int m_last_crc;
    unsigned int m_last_dbg;
    int goldo_fpga_send_spi_frame(void);
    void process_message();

    MessageQueue m_message_queue;
    unsigned char m_message_queue_buffer[128];

    uint32_t m_sensors_state{0};

    uint32_t m_last_timestamp{0};

    ServosConfig* m_servos_config;
    float m_servos_positions[16];
    uint16_t m_servos_speeds[16];
    uint16_t m_servos_target_positions[16];

    unsigned char m_crc_table[256];
    void generate_crc_table();
    unsigned char MyLittleCRC(unsigned char INCRC, unsigned char INBIT);
    unsigned char swap_bits(unsigned char IN);
    unsigned char CalculateCRC(unsigned char INCRC, unsigned char INBYTE);
    int goldo_fpga_check_crc(unsigned char *buf5, unsigned char recv_crc);
    void goldo_send_log_uint32(const char *msg, unsigned int val);

    int m_total_spi_frame_cnt;
    int m_strange_err_cnt;
    int m_addr1_crc_err_cnt;
    int m_addr2_crc_err_cnt;
    int m_write1_crc_err_cnt;
    int m_write2_crc_err_cnt;
    int m_read1_crc_err_cnt;
    int m_read2_crc_err_cnt;
    int m_apb_err_cnt;
  };
}
