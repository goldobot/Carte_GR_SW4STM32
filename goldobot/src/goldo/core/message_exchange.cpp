#include "goldo/core/message_exchange.hpp"

using namespace goldobot;

MessageExchange::MessageExchange()
{
  m_mutex = xSemaphoreCreateMutex();
}

bool MessageExchange::pushMessage(CommMessageType message_type, const unsigned char* buffer, size_t size, uint32_t seq)
{
  while(xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
  {
  };

  for(int i=0; i < m_num_subscriptions; i++)
  {
    auto& sub = m_subscriptions[i];
    if((uint16_t)message_type >= sub.message_type_first && (uint16_t)message_type <= sub.message_type_last)
    {
      sub.queue->push_message((uint16_t)message_type, buffer, size, (uint16_t)seq);
    }
  }
  xSemaphoreGive(m_mutex);
  return true;
}

void MessageExchange::subscribe(const Subscription& sub)
{
  while(xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE)
  {
  };

  m_subscriptions[m_num_subscriptions] = sub;
  m_num_subscriptions++;
  xSemaphoreGive(m_mutex);
}

