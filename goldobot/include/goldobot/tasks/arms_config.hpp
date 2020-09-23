#pragma once

#include <cstdint>

namespace goldobot {

struct DynamixelsConfig {
  uint16_t m_positions[3 * 32];
  uint16_t m_torque_settings[3 * 8];
};
}  // namespace goldobot
