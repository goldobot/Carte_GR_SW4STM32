#pragma once
#include <mutex>
namespace goldobot {
namespace detail {
struct LockerNull {
  void lock(){};
  void unlock(){};
};

using LockerMutex = std::mutex;
}  // namespace detail
}  // namespace goldobot
