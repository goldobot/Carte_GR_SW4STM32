#pragma once
#include "goldobot/platform/hal_private.hpp"
#include "stm32f3xx_hal.h"

extern "C"
{
	#include "stm32f3xx_hal_dma.h"
}

namespace goldobot { namespace platform {


extern DMA_HandleTypeDef g_dma_handles[12];

DMA_HandleTypeDef* hal_dma_init_device(DeviceId device, int signal, const DMA_InitTypeDef& init);



}} // namespace goldobot::platform
