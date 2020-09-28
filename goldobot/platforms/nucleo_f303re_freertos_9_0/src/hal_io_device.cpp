#include "goldobot/platform/hal_io_device.hpp"

#include "goldobot/platform/hal_i2c.hpp"
#include "goldobot/platform/hal_spi.hpp"
#include "goldobot/platform/hal_uart.hpp"

namespace goldobot {
namespace hal {
namespace platform {

IODevice g_io_devices[8];

void init_io_device(IODeviceConfig* config) {
  IODevice* device = &g_io_devices[config->io_device_id];
  device->io_flags = config->io_flags;

  if (config->rx_buffer_size > 0) {
    uint8_t* rx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
    device->rx_queue.init(rx_buffer, config->rx_buffer_size);
  }

  if (config->rx_buffer_size > 0) {
    uint8_t* tx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
    device->tx_queue.init(tx_buffer, config->tx_buffer_size);
  }

  device->rx_semaphore = xSemaphoreCreateBinary();
  device->tx_semaphore = xSemaphoreCreateBinary();

  switch (config->device_type) {
    case DeviceType::Uart:
      hal_usart_init(device, static_cast<const IODeviceConfigUart*>(config));
      break;
    case DeviceType::I2c:
      hal_i2c_init(device, static_cast<const IODeviceConfigI2c*>(config));
      break;
    case DeviceType::Spi:
      hal_spi_init(device, static_cast<const IODeviceConfigSpi*>(config));
      break;
    default:
      break;
  }

  // Non blocking io (fifo mode)
  if (!(device->io_flags & IODeviceFlags::RxBlocking)) {
    device->start_rx_fifo();
  }
}

void io_device_rx_complete_callback_fifo(IORequest* req, IODevice* device) {
  assert(req->state == IORequestState::RxComplete);
  device->rx_queue.unmap_push(req->rx_ptr, req->size - req->remaining);

  req->state = IORequestState::Ready;
  req->rx_ptr = nullptr;
  req->size = 0;
  req->remaining = 0;

  uint8_t* ptr;
  auto size = device->rx_queue.map_push(&ptr);

  if (size > 0) {
    req->rx_ptr = ptr;
    req->size = size;
    device->rx_functions->start_request(req, device->device_index);
  }
}

void io_device_tx_complete_callback_fifo(IORequest* req, IODevice* device) {
  assert(req->state == IORequestState::TxComplete);
  device->tx_queue.unmap_pop(req->tx_ptr, req->size - req->remaining);

  req->state = IORequestState::Ready;
  req->tx_ptr = nullptr;
  req->size = 0;
  req->remaining = 0;

  uint8_t* ptr;
  auto size = device->tx_queue.map_pop(&ptr);

  if (size > 0) {
    req->tx_ptr = ptr;
    req->size = size;
    device->tx_functions->start_request(req, device->device_index);
  }
}

void IODevice::start_rx_fifo() {
  uint8_t* ptr;
  auto size = rx_queue.map_push(&ptr);

  if (size > 0) {
    rx_request.rx_ptr = ptr;
    rx_request.size = size;
    rx_request.callback = &io_device_rx_complete_callback_fifo;
    hal_callback_send(HalCallback{DeviceType::IODevice, this - g_io_devices, 0});
  }
}

void IODevice::start_tx_fifo() {
  uint8_t* ptr;
  auto size = tx_queue.map_pop(&ptr);

  if (size > 0) {
    tx_request.tx_ptr = ptr;
    tx_request.size = size;
    tx_request.callback = &io_device_tx_complete_callback_fifo;
    hal_callback_send(HalCallback{DeviceType::IODevice, this - g_io_devices, 1});
  }
}

void io_device_rx_complete_callback_blocking(IORequest* req, IODevice* device) {
  assert(req->state == IORequestState::RxComplete);
  device->rx_queue.unmap_push(req->rx_ptr, req->size - req->remaining);

  req->state = IORequestState::Ready;
  req->rx_ptr = nullptr;
  req->size = 0;
  req->remaining = 0;
  xSemaphoreGive(device->rx_semaphore);
}

void io_device_tx_complete_callback_blocking(IORequest* req, IODevice* device) {
  assert(req->state == IORequestState::TxComplete);
  device->tx_queue.unmap_pop(req->tx_ptr, req->size - req->remaining);

  req->state = IORequestState::Ready;
  req->tx_ptr = nullptr;
  req->size = 0;
  req->remaining = 0;
  xSemaphoreGive(device->tx_semaphore);
}

size_t IODevice::read(uint8_t* buffer, size_t buffer_size) {
  if (buffer_size == 0) {
    return 0;
  }

  if (io_flags & IODeviceFlags::RxBlocking) {
    // blocking io
    rx_request.rx_ptr = buffer;
    rx_request.size = buffer_size;
    rx_request.callback = &io_device_rx_complete_callback_blocking;
    rx_functions->start_request(&rx_request, device_index);
    while (true) {
      xSemaphoreTake(rx_semaphore, portMAX_DELAY);
      if (rx_request.state == IORequestState::RxComplete) {
        rx_request.state = IORequestState::Ready;
        return rx_request.size - rx_request.remaining;
      }
      if (rx_request.state == IORequestState::Error) {
        rx_request.state = IORequestState::Ready;
        return 0;
      }
    }
  } else  // non blocking io
  {
    // Update state of rx request to read received data without waiting for end
    // of current transfer.
    taskENTER_CRITICAL();
    rx_functions->update_request(&rx_request, device_index);
    rx_queue.unmap_push(rx_request.rx_ptr, rx_request.size - rx_request.remaining);
    taskEXIT_CRITICAL();

    auto bytes_read = rx_queue.pop(buffer, buffer_size);

    taskENTER_CRITICAL();
    if (rx_request.state == IORequestState::Ready) {
      start_rx_fifo();
    }
    taskEXIT_CRITICAL();
    return bytes_read;
  }
  return 0;
}

size_t IODevice::write(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size == 0) {
    return 0;
  }
  if (io_flags & IODeviceFlags::TxBlocking) {
    // blocking io
    tx_request.tx_ptr = (uint8_t*)buffer;
    tx_request.size = buffer_size;
    tx_request.callback = &io_device_tx_complete_callback_blocking;
    tx_functions->start_request(&rx_request, device_index);
    while (true) {
      xSemaphoreTake(tx_semaphore, portMAX_DELAY);
      if (tx_request.state == IORequestState::TxComplete) {
        tx_request.state = IORequestState::Ready;
        return tx_request.size - tx_request.remaining;
      }
      if (tx_request.state == IORequestState::Error) {
        tx_request.state = IORequestState::Ready;
        return 0;
      }
    }
  } else {
    // non blocking io
    // Update state of tx request to get an accurate measure of space remaining
    // in the tx queue.
    taskENTER_CRITICAL();
    tx_functions->update_request(&tx_request, device_index);
    tx_queue.unmap_pop(tx_request.tx_ptr, tx_request.size - tx_request.remaining);
    taskEXIT_CRITICAL();

    uint16_t bytes_written = tx_queue.push(buffer, buffer_size);

    // If not currently transmitting, start tx request
    if (tx_request.state == IORequestState::Ready) {
      start_tx_fifo();
    }
    return bytes_written;
  }
  return 0;
}

size_t IODevice::map_read(uint8_t** buffer) {
  if (io_flags & IODeviceFlags::RxBlocking) {
    // mmapped io only works in fifo mode
    *buffer = nullptr;
    return 0;

  } else {
    // Update state of rx request to read received data without waiting for end
    // of current transfer.
    taskENTER_CRITICAL();
    if (rx_request.state == IORequestState::RxBusy) {
      rx_functions->update_request(&rx_request, device_index);
      rx_queue.unmap_push(rx_request.rx_ptr, rx_request.size - rx_request.remaining);
    }
    taskEXIT_CRITICAL();
    auto retval = rx_queue.map_pop(buffer);
    return retval;
  }
}

void IODevice::unmap_read(uint8_t* buffer, size_t size) {
  if (size != 0) {
    rx_queue.unmap_pop(buffer, size);
  }

  taskENTER_CRITICAL();
  if (rx_request.state == IORequestState::Ready) {
    start_rx_fifo();
  }
  taskEXIT_CRITICAL();
}

size_t IODevice::map_write(uint8_t** buffer) {
  if (io_flags & IODeviceFlags::RxBlocking) {
    // mmapped io only works in fifo mode
    *buffer = nullptr;
    return 0;

  } else {
    // Update state of tx request to get an accurate measure of space remaining
    // in the tx queue.
    taskENTER_CRITICAL();
    if (tx_request.state == IORequestState::TxBusy) {
      tx_functions->update_request(&tx_request, device_index);
      tx_queue.unmap_pop(tx_request.tx_ptr, tx_request.size - tx_request.remaining);
    }
    taskEXIT_CRITICAL();
    return tx_queue.map_push(buffer);
  }
}

void IODevice::unmap_write(uint8_t* buffer, size_t size) {
  tx_queue.unmap_push(buffer, size);

  // If not currently transmitting, start tx request
  if (tx_request.state == IORequestState::Ready) {
    start_tx_fifo();
  }
}

void hal_iodevice_callback(int id, int callback_id) {
  IODevice* device = &g_io_devices[id];
  switch (callback_id) {
    case 0:
      device->rx_functions->start_request(&device->rx_request, device->device_index);
      break;
    case 1:
      device->tx_functions->start_request(&device->tx_request, device->device_index);
      break;
  }
}

}  // namespace platform
using namespace platform;

static void check_io_device_id(int id) {
  assert(id >= 0 && (unsigned)id < sizeof(g_io_devices) / sizeof(IODevice));
}

size_t io_read(int id, uint8_t* buffer, size_t buffer_size) {
  check_io_device_id(id);
  return g_io_devices[id].read(buffer, buffer_size);
}

size_t io_write(int id, const uint8_t* buffer, size_t buffer_size) {
  check_io_device_id(id);
  return g_io_devices[id].write(buffer, buffer_size);
}

size_t io_map_read(int id, uint8_t** buffer) {
  check_io_device_id(id);
  return g_io_devices[id].map_read(buffer);
}

void io_unmap_read(int id, uint8_t* buffer, size_t size) {
  check_io_device_id(id);
  g_io_devices[id].unmap_read(buffer, size);
}

size_t io_map_write(int id, uint8_t** buffer) {
  check_io_device_id(id);
  return g_io_devices[id].map_write(buffer);
}

void io_unmap_write(int id, uint8_t* buffer, size_t size) {
  check_io_device_id(id);
  g_io_devices[id].unmap_write(buffer, size);
}

size_t io_read_bytes_available(int id) {
  check_io_device_id(id);
  return g_io_devices[id].rx_queue.size();
}

size_t io_write_space_available(int id) {
  auto& device = g_io_devices[id];
  return device.tx_queue.space_available();
}

}  // namespace hal
}  // namespace goldobot
