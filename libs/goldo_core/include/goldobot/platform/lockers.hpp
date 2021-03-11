#pragma once
#ifdef GOLDOBOT_PLATFORM_OS_FreeRTOS
#include "goldobot/platform/detail/freertos_lockers.hpp"
#else
#include "goldobot/platform/detail/stl_lockers.hpp"
#endif