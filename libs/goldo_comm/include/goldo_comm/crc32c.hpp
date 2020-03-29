#include <cstdint>
#include <cstddef>

namespace goldo_comm
{
    uint32_t crc32c(uint32_t crc, const uint8_t* data, size_t length);  
}
