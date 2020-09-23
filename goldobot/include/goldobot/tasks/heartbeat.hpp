#pragma once
#include <cstdint>

#include "goldobot/platform/task.hpp"

namespace goldobot {
class HeartbeatTask : public Task {
 public:
  HeartbeatTask();
  const char* name() const override;

 private:
  void taskFunction() override;
};
}  // namespace goldobot
