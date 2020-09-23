#pragma once
#include "goldobot/platform/hal_io_device.hpp"

namespace goldobot {
namespace hal {
namespace platform {

void hal_i2c_init(IODevice* device, const IODeviceConfigI2c* config);

}  // namespace platform
}  // namespace hal
};  // namespace goldobot
