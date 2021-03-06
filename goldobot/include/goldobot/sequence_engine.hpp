#pragma once
#include "goldobot/enums.hpp"
#include "goldobot/core/message_exchange.hpp"

#include <cstdint>

namespace goldobot {
struct Op {
  uint8_t opcode;
  uint8_t arg1;
  uint8_t arg2;
  uint8_t arg3;
};

enum class SequenceState : uint8_t { WaitForInit, Idle, Executing };

class SequenceEngine {
 public:
  SequenceEngine();

  void doStep(); /*< Execute instructions until forced to wait*/

  void setBuffer(unsigned char* buffer) { m_buffer = buffer; };
  void beginLoad();
  void loadData(unsigned char* data, uint16_t size);
  void endLoad();
  void startSequence(int id);
  void abortSequence();

  SequenceState state() const { return m_state; };

  void updateArmState(ArmState sta);
  void updatePropulsionState(PropulsionState state);
  void updateServoState(int id, bool moving);

 private:
  MessageExchange* m_exchange_commands;

  SequenceState m_state{SequenceState::WaitForInit};
  Op* m_ops{nullptr};
  uint16_t* m_sequence_offsets{nullptr};
  unsigned char* m_vars{nullptr};
  unsigned char* m_var_types;
  unsigned char* m_buffer;
  int m_load_offset{0};
  uint8_t m_num_vars;
  uint8_t m_num_seqs;

  uint16_t m_pc{0};
  uint16_t m_call_stack[8];
  int m_stack_level{0};
  uint32_t m_status_register{0};

  uint32_t m_end_delay{0};

  bool execOp(const Op& op);

  bool m_propulsion_state_dirty{false};
  PropulsionState m_propulsion_state{PropulsionState::Inactive};

  bool m_arm_state_dirty{false};
  ArmState m_arm_state{ArmState::Unconfigured};

  bool m_servo_state_dirty[16];
  bool m_servo_moving[16];

  float get_var_float(int index) { return *(float*)(m_vars + 4 * index); }

  // organisation of buffer:
  // header, 64 bits (uint16_t size, uint16_t crc16, uint8_t num_seq, uint8_tnum_var, reserved)
  // array of 32bits variables, 32bits aligned
  // array of 16bit sequence start offsets
  // array of 32bits ops, 32bits aligned
};

struct SequenceHeader {
  uint16_t size;
  uint16_t crc16;
  uint8_t num_vars;
  uint8_t num_seqs;
  uint8_t reserved[2];
};
}  // namespace goldobot
