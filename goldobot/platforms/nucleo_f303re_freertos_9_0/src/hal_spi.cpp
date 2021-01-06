#include "goldobot/platform/hal_spi.hpp"

#include "goldobot/platform/hal_gpio.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

extern "C" {
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"

void goldobot_hal_spi_irq_handler(int index);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
}

namespace goldobot {
namespace hal {
namespace platform {
SPI_HandleTypeDef g_spi_handles[3];
IODevice* g_spi_io_devices[3];

inline uint8_t get_spi_index(SPI_HandleTypeDef* hspi) {
  return static_cast<uint8_t>(hspi - g_spi_handles);
}

}  // namespace platform
}  // namespace hal
};  // namespace goldobot

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  using namespace goldobot::hal::platform;
  uint8_t spi_index = get_spi_index(hspi);
  auto io_device = g_spi_io_devices[spi_index];

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(io_device->tx_semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  using namespace goldobot::hal::platform;
  uint8_t spi_index = get_spi_index(hspi);
  auto io_device = g_spi_io_devices[spi_index];

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(io_device->tx_semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
// void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)

using namespace goldobot::hal::platform;

void goldobot_hal_spi_irq_handler(int index) { HAL_SPI_IRQHandler(&g_spi_handles[index]); }

namespace goldobot {
namespace hal {
namespace platform {

namespace {
int baudrate_prescaler_flag(int baudrate_prescaler) {
  switch (baudrate_prescaler) {
    case 2:
      return SPI_BAUDRATEPRESCALER_2;
    case 4:
      return SPI_BAUDRATEPRESCALER_4;
    case 8:
      return SPI_BAUDRATEPRESCALER_8;
    case 16:
      return SPI_BAUDRATEPRESCALER_16;
    case 32:
      return SPI_BAUDRATEPRESCALER_32;
    case 64:
      return SPI_BAUDRATEPRESCALER_64;
    case 128:
      return SPI_BAUDRATEPRESCALER_128;
    case 256:
      return SPI_BAUDRATEPRESCALER_256;
    default:
      assert(false);
  }
}
}  // namespace

void hal_spi_init(IODevice* device, const IODeviceConfigSpi* config) {
  int spi_index = (int)config->device_id - (int)DeviceId::Spi1;

  IRQn_Type irq_n;
  SPI_HandleTypeDef* spi_handle = &g_spi_handles[spi_index];
  SPI_TypeDef* spi_instance;

  switch (config->device_id) {
    case DeviceId::Spi1:
      spi_instance = SPI1;
      irq_n = SPI1_IRQn;
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
      break;

    case DeviceId::Spi2:
      spi_instance = SPI2;
      irq_n = SPI2_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
      break;
    case DeviceId::Spi3:
      spi_instance = SPI3;
      irq_n = SPI3_IRQn;
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
      break;
    default:
      break;
  }

  g_spi_io_devices[spi_index] = device;
  device->device_index = spi_index;
  // device->functions = &g_spi_device_functions;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  hal_gpio_init_pin_af(config->device_id, 0, config->sck_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 1, config->mosi_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 2, config->miso_pin, GPIO_InitStruct);
  hal_gpio_init_pin_af(config->device_id, 3, config->nss_pin, GPIO_InitStruct);

  // SPI interrupt Init
  HAL_NVIC_SetPriority(irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(irq_n);

  spi_handle->Instance = spi_instance;
  spi_handle->Init.Mode = SPI_MODE_MASTER;
  spi_handle->Init.Direction = SPI_DIRECTION_2LINES;
  spi_handle->Init.DataSize = SPI_DATASIZE_8BIT;
  spi_handle->Init.CLKPolarity = SPI_POLARITY_LOW;
  spi_handle->Init.CLKPhase = SPI_PHASE_1EDGE;
  spi_handle->Init.NSS = SPI_NSS_SOFT;
  spi_handle->Init.BaudRatePrescaler = baudrate_prescaler_flag(config->baudrate_prescaler);
  spi_handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_handle->Init.TIMode = SPI_TIMODE_DISABLE;

  // no hardware crc calculation
  spi_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi_handle->Init.CRCPolynomial = 7;
  spi_handle->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;

  spi_handle->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(spi_handle);
}

}  // namespace platform

void spi_read_write(int id, uint8_t* read_buffer, const uint8_t* write_buffer, size_t size) {
  auto hspi = &g_spi_handles[g_io_devices[id].device_index];
  auto& io_device = g_io_devices[id];
  HAL_SPI_TransmitReceive_IT(hspi, const_cast<uint8_t*>(write_buffer), read_buffer, size);
  while (xSemaphoreTake(io_device.tx_semaphore, portMAX_DELAY) != pdTRUE &&
         HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
  }
}

}  // namespace hal
};  // namespace goldobot
