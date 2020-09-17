#pragma once
#include "goldobot/platform/hal_private.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot { namespace platform {

void hal_i2c_init(IODevice* device, const IODeviceConfigI2c* config);

} };
