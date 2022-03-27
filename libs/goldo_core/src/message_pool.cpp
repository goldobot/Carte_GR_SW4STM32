#include "goldobot/core/message_pool.hpp"

#include <mutex>
#include <cassert>
#include <cstring>
#include <atomic>
#include<memory>

namespace goldobot {
class MessagePool::ControlBlock  {
public:
	void incRef();
	void decRef();

	uint8_t* mData;
	uint16_t mMessageType;
	uint16_t mMessageSize;
	// high 2 bytes = offset between data and arena control block, bottom 2 bytes = reference counter
	std::atomic<uint32_t> mCounterAndArenaOffset;
};

void MessagePool::ControlBlock::incRef() {
	mCounterAndArenaOffset.fetch_add(1);
}

void MessagePool::ControlBlock::decRef() {
	if(mCounterAndArenaOffset.fetch_add(-1) == 1) {
		std::atomic_thread_fence(std::memory_order_acquire);
		// return to arena
		Arena* arena = reinterpret_cast<Arena*>(mData - ((mCounterAndArenaOffset & 0xff00) >> 16));
		arena->deallocate(this);
	}
}

MessagePool::Handle::Handle() : mControlBlock(nullptr) {};

MessagePool::Handle::Handle(ControlBlock* cb) : mControlBlock(cb) {
	mControlBlock->incRef();
}

MessagePool::Handle::Handle(const Handle& from) {
	mControlBlock = from.mControlBlock;
	if(mControlBlock != nullptr)
	mControlBlock->incRef();
}

MessagePool::Handle::Handle(Handle&& from) {
	std::swap(mControlBlock, from.mControlBlock);
}

MessagePool::Handle::~Handle() {
	if(mControlBlock)
		mControlBlock->decRef();
}

MessagePool::Handle& MessagePool::Handle::operator=(const Handle& from) {
	return *this;
}

MessagePool::Handle& MessagePool::Handle::operator==(Handle&& from) {
	return *this;
}


CommMessageType MessagePool::Handle::type() const noexcept { return static_cast<CommMessageType>(mControlBlock->mMessageType);};
		size_t MessagePool::Handle::size() const noexcept {
			return mControlBlock->mMessageSize;
		}
uint8_t* MessagePool::Handle::data() noexcept {
		return mControlBlock->mData;
};


}  // namespace goldobot
