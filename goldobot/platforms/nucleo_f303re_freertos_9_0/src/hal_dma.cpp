#include "goldobot/platform/hal_dma.hpp"

namespace goldobot {
namespace hal {
namespace platform {

DMA_HandleTypeDef g_dma_handles[12];

DMA_Channel_TypeDef* g_dma_channels[12] = {
    DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6,
    DMA1_Channel7, DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5};

IRQn_Type g_dma_irq_numbers[12] = {DMA1_Channel1_IRQn,
                                   DMA1_Channel2_IRQn,
                                   DMA1_Channel3_IRQn,
                                   DMA1_Channel4_IRQn,
                                   DMA1_Channel5_IRQn,
                                   DMA1_Channel6_IRQn,
                                   DMA1_Channel7_IRQn,
                                   DMA2_Channel1_IRQn,
                                   DMA2_Channel2_IRQn,
                                   DMA2_Channel3_IRQn,
                                   DMA2_Channel4_IRQn,
                                   DMA2_Channel5_IRQn

};

struct DMAReference {
  DeviceId device;
  uint8_t signal;
  uint8_t dma_index;
};

const DMAReference g_dma_references[] = {{DeviceId::Usart1, 0, 4}, {DeviceId::Usart1, 1, 3},
                                         {DeviceId::Usart2, 0, 5}, {DeviceId::Usart2, 1, 6},
                                         {DeviceId::Usart3, 0, 2}, {DeviceId::Usart3, 1, 1},
                                         {DeviceId::Uart4, 0, 9},  {DeviceId::Uart4, 1, 11}};
}  // namespace platform
}  // namespace hal
}  // namespace goldobot
using namespace goldobot::hal::platform;

extern "C" {
void goldobot_hal_dma_irq_handler(int dma_index);
}

void goldobot_hal_dma_irq_handler(int dma_index) { HAL_DMA_IRQHandler(&g_dma_handles[dma_index]); }

namespace goldobot {
namespace hal {
namespace platform {

DMA_HandleTypeDef* hal_dma_init_device(DeviceId device, int signal, const DMA_InitTypeDef& init) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  for (int i = 0; i < 12; i++) {
    auto dma_ref = &g_dma_references[i];
    if (dma_ref->device == device && dma_ref->signal == signal) {
      int dma_index = dma_ref->dma_index;
      IRQn_Type irq_n = g_dma_irq_numbers[dma_index];
      HAL_NVIC_SetPriority(irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
      HAL_NVIC_EnableIRQ(irq_n);

      DMA_HandleTypeDef* handle = &g_dma_handles[dma_index];

      // break if the dma channel is already used by another device
      assert(handle->Parent == nullptr);

      handle->Instance = g_dma_channels[dma_index];
      handle->Init = init;
      HAL_DMA_Init(handle);
      return handle;
    }
  }
  return nullptr;
}

}  // namespace platform
}  // namespace hal
};  // namespace goldobot
