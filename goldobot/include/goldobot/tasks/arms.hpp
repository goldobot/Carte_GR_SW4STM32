#pragma once
#include <cstdint>

#include "goldobot/config.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"

namespace goldobot {
struct DynamixelsConfig {
  uint16_t m_positions[3 * 32];
  uint16_t m_torque_settings[3 * 8];
};

enum class DynamixelStatusError { Ok, ChecksumError, TimeoutError };

struct DynamixelState {
  uint16_t position;
  uint16_t speed;
  uint16_t torque;
};

class ArmsTask : public Task {
 public:
  ArmsTask();
  const char* name() const override;

  // Dirty provisional api

  void go_to_position(uint8_t pos_id, uint16_t speed_percent, int torque_setting = 0);
  // Disable all dynamixels torque
  void shutdown();

  //! \brief Read data from dynamixel registers
  bool dynamixels_read_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
  bool dynamixels_write_data(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
  // \brief Write data to dynamixel buffer. data is transferred to control registers on ACTION
  // packet
  bool dynamixels_reg_write(uint8_t id, uint8_t address, unsigned char* buffer, uint8_t size);
  void dynamixels_action();

  // void _execute_command(int arm_id, const ArmCommand& command);

  // private:
  void dynamixels_transmit_packet(uint8_t id, uint8_t command, unsigned char* parameters,
                                  uint8_t num_parameters);
  DynamixelStatusError dynamixels_receive_packet();
  void taskFunction() override;

  ArmConfig m_config;

  uint16_t m_current_position[8];
  uint16_t m_current_load[8];
  DynamixelState m_current_state[8];

  unsigned char m_dynamixels_buffer[256];
  bool m_dynamixels_receive_ok;
  uint8_t m_dynamixels_receive_id;
  uint8_t m_dynamixels_receive_num_parameters;
  uint8_t m_dynamixels_receive_error;
  uint8_t* m_dynamixels_receive_params;

  MessageQueue m_message_queue;
  unsigned char m_message_queue_buffer[256];

  // uint16_t m_dynamixels_positions[3];
  // uint16_t m_dynamixels_loads[3];

  uint32_t m_end_move_timestamp;
  ArmState m_arm_state{ArmState::Unconfigured};

  void process_message();
};
}  // namespace goldobot
