#pragma once
#include <array>
#include <cstdint>

namespace goldobot {

enum class CommMessageType : uint16_t;

class MessagePool {
 public:
  class ControlBlock;
  class Arena;

  class Handle {
   public:
    Handle();
    Handle(ControlBlock* cb);
    Handle(const Handle& from);
    Handle(Handle&& from);
    ~Handle();
    Handle& operator=(const Handle& from);
    Handle& operator==(Handle&& from);

    CommMessageType type() const noexcept;
    size_t size() const noexcept;
    uint8_t* data() noexcept;

   private:
    ControlBlock* mControlBlock;
  };

  class Arena {
   public:
    ControlBlock* allocate();
    void deallocate(ControlBlock* cb);
  };
};

}  // namespace goldobot
