#pragma once
#include "goldobot/platform/message_queue.hpp"

#include <array>

namespace goldobot
{
	enum class CommMessageType : uint16_t;

	class MessageExchange
	{
	public:
		/**
		 * Subscription slot.
		 * Route message with type between first and last to queue.
		 */
		struct Subscription
		{
			uint16_t message_type_first;
			uint16_t message_type_last;
			MessageQueue* queue;
		};
	public:
		MessageExchange();

		bool pushMessage(CommMessageType message_type, const unsigned char* buffer, size_t size);
		void subscribe(const Subscription& sub);

	private:
		std::array<Subscription, 64> m_subscriptions;
		int m_num_subscriptions{0};
		mutable std::mutex m_mutex;
	};
}
