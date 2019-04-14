#include "goldobot/core/message_exchange.hpp"

using namespace goldobot;

MessageExchange::MessageExchange()
{
	m_mutex = xSemaphoreCreateMutex();
}

bool MessageExchange::pushMessage(uint16_t message_type, const unsigned char* buffer, size_t size)
{
	for(int i=0; i < m_num_subscriptions; i++)
	{
		auto& sub = m_subscriptions[i];
		if(message_type >= sub.message_type_first && message_type <= sub.message_type_last)
		{
			sub.queue->push_message(message_type, buffer, size);
		}
	}
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

