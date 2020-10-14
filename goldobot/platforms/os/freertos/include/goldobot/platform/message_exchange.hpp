#pragma once
#include "goldobot/platform/message_queue.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <cstdint>

namespace goldobot {
class MessageExchange {
 public:
  /**
   * Subscription slot.
   * Route message with type between first and last to queue.
   */
  struct Subscription {
    uint16_t message_type_first;
    uint16_t message_type_last;
    MessageQueue* queue;
  };

 public:
  MessageExchange();

  bool pushMessage(CommMessageType message_type, const unsigned char* buffer, size_t size);
  void subscribe(const Subscription& sub);

 private:
  Subscription m_subscriptions[64];
  int m_num_subscriptions{0};
  SemaphoreHandle_t m_mutex;
};
}  // namespace goldobot
