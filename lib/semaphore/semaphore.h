#ifndef LIB_SEMAPHORE_SEMAPHORE_H_
#define LIB_SEMAPHORE_SEMAPHORE_H_

#include <mutex>
#include <condition_variable>

namespace lib {

class Semaphore {
 public:
  Semaphore(unsigned long count);
  void Notify();
  void Wait();
  bool TryWait();

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned long count_;
};

}  // namespace lib

#endif  // LIB_SEMAPHORE_SEMAPHORE_H_
