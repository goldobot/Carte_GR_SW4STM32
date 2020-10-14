#pragma once
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {
class HeartbeatTask : public Task {
 public:
  HeartbeatTask();
  const char* name() const override;

 private:
  void taskFunction() override;
};
}  // namespace goldobot
