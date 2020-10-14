#include "goldobot/core/message_exchange.hpp"

using namespace goldobot;

MessageExchange::MessageExchange() {}

bool MessageExchange::pushMessage(CommMessageType message_type, const unsigned char* buffer,
                                  size_t size) {
  std::lock_guard<std::mutex> lck(m_mutex);

  for (int i = 0; i < m_num_subscriptions; i++) {
    auto& sub = m_subscriptions[i];
    if ((uint16_t)message_type >= sub.message_type_first &&
        (uint16_t)message_type <= sub.message_type_last) {
      sub.queue->push_message((uint16_t)message_type, buffer, size);
    }
  }
  return true;
}

void MessageExchange::subscribe(const Subscription& sub) {
  std::lock_guard<std::mutex> lck(m_mutex);

  m_subscriptions[m_num_subscriptions] = sub;
  m_num_subscriptions++;
}
