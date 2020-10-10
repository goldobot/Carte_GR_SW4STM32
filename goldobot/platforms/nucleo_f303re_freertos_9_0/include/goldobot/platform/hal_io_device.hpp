#pragma once
#include <assert.h>

#include <atomic>

#include "goldobot/hal.hpp"
#include "goldobot/platform/hal_io_device_queue.hpp"
#include "goldobot/platform/hal_private.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot {
namespace hal {
namespace platform {

struct IORequest;
struct IODevice;

typedef void (*IORequestCallback)(IORequest*, IODevice* device);
typedef void (*IORequestFunction)(IORequest*, uint32_t device_index);

enum class IORequestState : uint32_t { Ready, Pending, Busy, Complete, Error };

struct IORequest {
  uint8_t* rx_ptr;
  uint8_t* tx_ptr;
  uint32_t size;
  std::atomic<uint32_t> remaining;
  std::atomic<IORequestState> state{IORequestState::Ready};
  IORequestCallback callback;
};

struct IODeviceFunctions {
  IORequestFunction start_request;
  IORequestFunction update_request;
  IORequestFunction abort_request;
};

class IODevice {
 public:
  size_t read(uint8_t* buffer, size_t buffer_size);
  size_t write(const uint8_t* buffer, size_t buffer_size);

  size_t map_read(uint8_t** buffer);
  void unmap_read(uint8_t* buffer, size_t size);

  size_t map_write(uint8_t** buffer);
  void unmap_write(uint8_t* buffer, size_t size);

  void try_start_rx_fifo();
  void start_tx_fifo();

  void schedule_callback(uint8_t callback_id);
  bool queue_rx_request(uint8_t* buffer, size_t size, IORequestCallback callback);

  uint32_t device_index;  // index of raw device (0 for UART1 for example)
  uint16_t io_flags;
  PinID txen_pin;  // gpio pin that is set to 1 when transmitting

  IODeviceFunctions* rx_functions;
  IODeviceQueue rx_queue;
  IORequest rx_request;
  IORequest rx_request_next;
  uint8_t* rx_next_head{nullptr};
  SemaphoreHandle_t rx_semaphore;

  IODeviceFunctions* tx_functions;
  IODeviceQueue tx_queue;
  IORequest tx_request;
  SemaphoreHandle_t tx_semaphore;
};

void init_io_device(IODeviceConfig* config);

extern IODevice g_io_devices[8];

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
