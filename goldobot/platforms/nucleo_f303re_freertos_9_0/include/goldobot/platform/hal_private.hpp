#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

namespace goldobot {
namespace hal {
namespace platform {

enum class DeviceType : uint8_t {
  None = 0,
  Gpio,
  Timer,
  Pwm,
  Encoder,
  Uart,
  I2c,
  Spi,
  Can,
  IODevice
};

enum class DeviceId : uint8_t {
  Gpio = 1,
  Tim1,
  Tim2,
  Tim3,
  Tim4,
  Tim6,
  Tim7,
  Tim8,
  Tim15,
  Tim16,
  Tim17,
  Tim20,
  Can,
  I2c1,
  I2c2,
  I2c3,
  Spi1,
  Spi2,
  Spi3,
  Usart1,
  Usart2,
  Usart3,
  Uart4,
  Uart5
};

struct IODeviceFlags {
  static constexpr uint16_t RxBlocking = 0x01;
  static constexpr uint16_t TxBlocking = 0x02;
  static constexpr uint16_t RxDma = 0x04;
  static constexpr uint16_t TxDma = 0x08;
  static constexpr uint16_t IdleInterrupt = 0x0100;
};

struct EncoderFlags {
  static constexpr uint8_t ReverseDirection = 0x01;
  static constexpr uint8_t HallMode =
      0x02;  // CH1 is replaced by XOR of CH1, CH2, CH3, used for hall encoders
};

struct GpioFlags {
  static constexpr uint8_t ModeInput = 0x00;
  static constexpr uint8_t ModetPushPull = 0x01;
  static constexpr uint8_t ModeOutputOpenDrain = 0x02;
};

struct PinID {
  uint8_t port{0xff};
  uint8_t pin{0xff};
};

struct DeviceConfig {
  uint16_t struct_size;
  DeviceType device_type;
  DeviceId device_id;
};

struct DeviceConfigTimer : DeviceConfig {
  uint32_t prescaler;
  uint32_t period;
};

struct DeviceConfigPwm : DeviceConfig {
  uint8_t pwm_id;
  uint8_t channel;
  PinID pin;
  PinID n_pin;
  PinID dir_pin;
};

struct DeviceConfigEncoder : DeviceConfig {
  uint8_t tim_index;
  uint8_t flags;
  PinID ch1_pin;
  PinID ch2_pin;
  PinID ch3_pin;
  uint16_t period;
};

struct IODeviceConfig : DeviceConfig {
  uint8_t io_device_id;
  uint8_t reserved;
  uint16_t rx_buffer_size;
  uint16_t tx_buffer_size;
  uint16_t io_flags;
};

struct IODeviceConfigUart : IODeviceConfig {
  uint32_t baudrate;
  PinID rx_pin;
  PinID tx_pin;
  PinID txen_pin;  // gpio pin that is set to 1 when transmitting
};

struct IODeviceConfigI2c : IODeviceConfig {
  PinID scl_pin;
  PinID sda_pin;
  uint32_t timing;
};

struct IODeviceConfigSpi : IODeviceConfig {
  PinID sck_pin;
  PinID mosi_pin;
  PinID miso_pin;
  PinID nss_pin;
  uint16_t baudrate_prescaler;
};
struct GpioDevice {
  uint8_t port;
  uint8_t pin;
};

struct HalCallback {
  DeviceType device_type;
  uint8_t device_index;
  uint8_t callback_index;
  uint8_t reserved;
};

void hal_callback_send(const HalCallback& callback);
void hal_callback_send_from_isr(const HalCallback& callback);
void hal_callback_handler_task_start();

void hal_iodevice_callback(int id, int callback_id);
void hal_uart_callback(int uart_index, int callback_id);

// GPIO helpers

// put it here for now

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
