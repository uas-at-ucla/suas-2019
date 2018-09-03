#pragma once

#include <condition_variable>
#include <mutex>

namespace lib {

class Semaphore {
 public:
  Semaphore(unsigned long count);
  void Notify();
  void Wait();
  bool TryWait();

 private:
  ::std::mutex mutex_;
  ::std::condition_variable condition_;
  unsigned long count_;
};

} // namespace lib