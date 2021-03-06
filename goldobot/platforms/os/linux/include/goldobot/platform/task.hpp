#pragma once
#include <atomic>
#include <chrono>
#include <thread>

typedef uint32_t TickType_t;

namespace goldobot {
class Task {
 public:
  enum State { Uninitialized, Stopped, Running };

 public:
  virtual ~Task();
  virtual const char* name() const = 0;

  State state() const;

  void init();
  void start();
  void stop();

 protected:
  Task();
  virtual void taskFunction() = 0;

  void set_priority(unsigned prio);
  void delay(unsigned ticks);
  void delay_periodic(unsigned ticks);
  void checkStateUpdate();

  std::atomic<State> m_state{Task::Uninitialized};
  std::atomic<State> m_requested_state{Task::Uninitialized};

 private:
  std::thread m_thread;
  std::chrono::time_point<std::chrono::steady_clock> m_last_wake_time;
};
}  // namespace goldobot
