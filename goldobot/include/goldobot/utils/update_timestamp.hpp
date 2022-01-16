#include <cstddef>
#include <cstdint>

namespace goldobot {
inline void update_timestamp(uint32_t& timestamp, uint32_t now, uint32_t period) {
  uint32_t ts = timestamp;
  while (ts <= now) {
    ts += period;
  }
  timestamp = ts;
};

}  // namespace goldobot
