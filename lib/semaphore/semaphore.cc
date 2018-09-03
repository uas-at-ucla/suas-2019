#include "semaphore.h"

namespace lib {

Semaphore::Semaphore(unsigned long count) : count_(count) {}

void Semaphore::Notify() {
  ::std::unique_lock<decltype(mutex_)> lock(mutex_);
  count_++;
  condition_.notify_one();
}

void Semaphore::Wait() {
  ::std::unique_lock<decltype(mutex_)> lock(mutex_);

  while (!count_) {
    // Handle spurious wake-ups.
    condition_.wait(lock);
  }

  count_--;
}

bool Semaphore::TryWait() {
  ::std::unique_lock<decltype(mutex_)> lock(mutex_);
  if (count_) {
    count_--;
    return true;
  }

  return false;
}

} // namespace lib
