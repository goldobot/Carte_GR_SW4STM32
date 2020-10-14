#pragma once
#include "goldobot/platform/task.hpp"

namespace goldobot {
class PeriodicTask : public Task {
 public:
 public:
  PeriodicTask();
  virtual ~PeriodicTask();

 protected:
  virtual void taskFunction();
};
}  // namespace goldobot
