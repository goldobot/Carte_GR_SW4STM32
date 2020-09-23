#pragma once

#include <cstddef>
#include <cstdint>

namespace goldobot {

class DynamixelsInterface {
 public:
  enum class State {
    Inactive = 0,
    Stopped = 1,
    Moving = 2,
  };

 public:
  void reset(uint32_t time_ms);
  void update(uint32_t time_ms);
  void execute_movement(uint8_t target_position);

 private:
  void dynamixels_transmit_packet(uint8_t id, uint8_t command, unsigned char* parameters,
                                  uint8_t num_parameters);
  bool dynamixels_receive_packet();
};

}  // namespace goldobot
