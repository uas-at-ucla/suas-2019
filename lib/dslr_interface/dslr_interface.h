#ifndef LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_
#define LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_

#include <atomic>
#include <mutex>
#include <condition_variable>

namespace lib {

class DSLRInterface {
 public:
  DSLRInterface();

  enum State {
    STANDBY,
    CONTINUOUSLY_TAKE_PHOTOS,
    DOWNLOAD_PHOTOS,
    STOP_DOWNLOADING_PHOTOS,
    WAIT_FOR_DOWNLOADING_PHOTOS_EXIT
  };

 private:
  State state_;

  ::std::thread thread_;
  ::std::atomic<bool> run_{true};

  void Run();
  void RunIteration();

  bool photos_capturing_;

  pid_t photos_capture_pid_;
  pid_t photos_download_pid_;

  double take_photos_triggered_;
};

}  // namespace lib

#endif  // LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_
