#pragma once
#include "goldobot/hal.hpp"
#include "goldobot/platform/hal_io_device_queue.hpp"
#include "goldobot/platform/hal_private.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

#include <atomic>

namespace goldobot {
namespace hal {
namespace platform {

struct IODevice;

//typedef void (*IORequestCallback)(IORequest*, IODevice* device);


enum class IORequestState : uint32_t { Ready, Pending, Busy, Complete, Error };

struct IORequestImpl : IORequest {
  std::atomic<uint32_t> remaining;
  std::atomic<IORequestState> state{IORequestState::Ready};
};

typedef void (*IORequestFunction)(IORequestImpl*, uint32_t device_index);

struct IODeviceFunctions {
  IORequestFunction start_request;
  IORequestFunction update_request;
  IORequestFunction abort_request;
};

class IODevice {
 public:
  void execute(IORequest request, uint32_t timeout);
  size_t read(uint8_t* buffer, size_t buffer_size, uint32_t timeout);
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

  IODeviceFunctions* rx_functions;
  IODeviceQueue rx_queue;
  IORequestImpl rx_request;

  IODeviceFunctions* tx_functions;
  IODeviceQueue tx_queue;
  IORequestImpl tx_request;

  SemaphoreHandle_t req_finished_semaphore;
};

void init_io_device(IODeviceConfig* config);

extern IODevice g_io_devices[8];

}  // namespace platform
}  // namespace hal
}  // namespace goldobot
