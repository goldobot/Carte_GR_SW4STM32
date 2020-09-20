#include "goldobot/platform/hal_dma.hpp"



namespace goldobot { namespace platform {

DMA_HandleTypeDef g_dma_handles[12];
DMA_Channel_TypeDef* g_dma_channels[12] = {
	DMA1_Channel1,
	DMA1_Channel2,
	DMA1_Channel3,
	DMA1_Channel4,
	DMA1_Channel5,
	DMA1_Channel6,
	DMA1_Channel7,
	DMA2_Channel1,
	DMA2_Channel2,
	DMA2_Channel3,
	DMA2_Channel4,
	DMA2_Channel5
};

struct DMAReference
{
	DeviceId device;
	uint8_t signal;
	uint8_t dma_index;
};

const DMAReference g_dma_references[] = {
		{DeviceId::Usart1, 0, 4},
		{DeviceId::Usart1, 1, 3},
		{DeviceId::Usart2, 0, 5},
		{DeviceId::Usart2, 1, 5},
		{DeviceId::Usart3, 0, 2},
		{DeviceId::Usart3, 1, 1},
		{DeviceId::Uart4, 0, 9},
		{DeviceId::Uart4, 1, 11}
};


DMA_HandleTypeDef* hal_dma_init_device(DeviceId device, int signal, const DMA_InitTypeDef& init)
{
	for(int i = 0; i < 8; i++)
	{
		auto dma_ref = &g_dma_references[i];
		if(dma_ref->device == device && dma_ref->signal == signal)
		{
			DMA_HandleTypeDef* handle = &g_dma_handles[dma_ref->dma_index];
			handle->Instance = g_dma_channels[dma_ref->dma_index];
			handle->Init = init;
			HAL_DMA_Init(handle);
			return handle;
		}
	}
	return nullptr;

}

} }; // namespace goldobot::platform
