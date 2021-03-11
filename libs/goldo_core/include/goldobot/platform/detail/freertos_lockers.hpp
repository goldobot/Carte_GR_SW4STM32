#include "FreeRTOS.h"
#include "semphr.h"

namespace goldobot {
namespace detail {
struct LockerNull {
  void lock(){};
  void unlock(){};
};

struct LockerMutex {
  LockerMutex() : m_handle(xSemaphoreCreateBinary()){};
  ~LockerMutex() { vSemaphoreDelete(m_handle); };

  void lock() {
    while (xSemaphoreTake(m_handle, portMAX_DELAY) != pdTRUE) {
    };
  };

  void unlock() { xSemaphoreGive(m_handle); };

 private:
  SemaphoreHandle_t m_handle;
};
}  // namespace detail
}  // namespace goldobot
