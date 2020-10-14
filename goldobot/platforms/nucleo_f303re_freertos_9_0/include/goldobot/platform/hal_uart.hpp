#pragma once
#include "goldobot/platform/hal_io_device.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot {
namespace hal {
namespace platform {

void hal_usart_init(IODevice* device, const IODeviceConfigUart* config);

}
}  // namespace hal
}  // namespace goldobot
