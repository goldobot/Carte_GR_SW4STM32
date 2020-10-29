#include "goldobot/platform/hal_gpio.hpp"
#include "goldobot/platform/hal_i2c.hpp"
#include "goldobot/platform/hal_io_device.hpp"
#include "goldobot/platform/hal_private.hpp"
#include "goldobot/platform/hal_spi.hpp"
#include "goldobot/platform/hal_timer.hpp"
#include "goldobot/platform/hal_uart.hpp"

#include "stm32f3xx_hal.h"
#include "core_cm4.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <errno.h>
#include <math.h>
#include <sys/unistd.h>  // STDOUT_FILENO, STDERR_FILENO

#include <algorithm>
#include <cstring>

// Configuration structures

namespace goldobot {
namespace hal {
using namespace platform;

void configure(uint8_t* config) {
  uint16_t num_devices = *reinterpret_cast<uint16_t*>(config);
  uint16_t* offsets = reinterpret_cast<uint16_t*>(config + 2);
  for (int i = 0; i < num_devices; i++) {
    DeviceConfig* device_config = reinterpret_cast<DeviceConfig*>(config + offsets[i]);
    switch (device_config->device_type) {
      case DeviceType::Gpio:
        hal_gpio_init(static_cast<DeviceConfigGpio*>(device_config));
        break;
      case DeviceType::Timer:
        hal_timer_init(static_cast<DeviceConfigTimer*>(device_config));
        break;
      case DeviceType::Pwm:
        hal_pwm_init(static_cast<DeviceConfigPwm*>(device_config));
        break;
      case DeviceType::Encoder:
        hal_encoder_init(static_cast<DeviceConfigEncoder*>(device_config));
        break;
      case DeviceType::Uart:
        init_io_device(static_cast<IODeviceConfig*>(device_config));
        break;
      case DeviceType::I2c:
        init_io_device(static_cast<IODeviceConfig*>(device_config));
        break;
      case DeviceType::Spi:
        init_io_device(static_cast<IODeviceConfig*>(device_config));
        break;
      default:
        break;
    }
  }
}
void init() {
  hal_callback_handler_task_start();

  // init uart
  IODeviceConfigUart uart_config;
  uart_config.device_type = DeviceType::Uart;
  uart_config.io_device_id = 0;
  uart_config.device_id = DeviceId::Usart2;
  uart_config.baudrate = 500000U;  // 230400U;
  uart_config.rx_pin = PinID{0, 3};
  uart_config.tx_pin = PinID{0, 2};
  uart_config.rx_buffer_size = 256;
  uart_config.tx_buffer_size = 256;
  uart_config.io_flags = IODeviceFlags::RxDma | IODeviceFlags::TxDma;

  init_io_device(&uart_config);
}

TickType_t get_tick_count() { return xTaskGetTickCount(); }
}  // namespace hal
}  // namespace goldobot
