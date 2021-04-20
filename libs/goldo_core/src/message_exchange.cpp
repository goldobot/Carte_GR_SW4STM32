#include "goldobot/core/message_exchange.hpp"

#include <cassert>
#include <mutex>

namespace goldobot {

MessageExchange::MessageExchange() {}
MessageExchange::~MessageExchange(){};

bool MessageExchange::pushMessage(CommMessageType message_type, const unsigned char* buffer,
                                  size_t size) {
  std::unique_lock<detail::LockerMutex>(m_mutex);

  for (int i = 0; i < m_num_subscriptions; i++) {
    auto& sub = m_subscriptions[i];
    if ((uint16_t)message_type >= sub.message_type_first &&
        (uint16_t)message_type <= sub.message_type_last) {
      sub.queue->push_message(message_type, buffer, size);
    }
  }
  return true;
}

void MessageExchange::subscribe(const Subscription& sub) {
  assert(sub.queue != nullptr);
  std::unique_lock<detail::LockerMutex>(m_mutex);
  m_subscriptions[m_num_subscriptions] = sub;
  m_num_subscriptions++;
}
}  // namespace goldobot
