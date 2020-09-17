#pragma once
#include "goldobot/platform/hal_private.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot { namespace platform {

void hal_usart_init(IODevice* device, const IODeviceConfigUART* config);

} };
