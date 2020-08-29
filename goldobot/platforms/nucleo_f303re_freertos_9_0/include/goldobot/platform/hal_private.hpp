#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

namespace goldobot
{
class HalPrivate
{
public:




	static HalPrivate s_instance;
};



} // namespace goldobot
