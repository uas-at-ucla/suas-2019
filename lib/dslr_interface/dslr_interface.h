#ifndef LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_
#define LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <string.h>

#include "aos/common/util/phased_loop.h"

#include <iostream>

namespace lib {

class DSLRInterface {
 public:
  DSLRInterface();

  enum State {
    STANDBY = 0,
    EXITED,
    START_CONTINUOUS_PHOTO_CAPTURE,
    CONTINUOUS_PHOTO_CAPTURE,
    STOP_CONTINUOUS_PHOTO_CAPTURE,
    WAIT_FOR_PHOTOS_CAPTURE_EXIT,
    START_DOWNLOAD_PHOTOS,
    DOWNLOAD_PHOTOS,
    STOP_DOWNLOADING_PHOTOS,
    WAIT_FOR_DOWNLOADING_PHOTOS_EXIT
  };

  void Quit();
  void TakePhotos();

 private:
  State state_;

  ::std::thread thread_;
  ::aos::time::PhasedLoop phased_loop_;

  ::std::atomic<bool> exiting_{false};
  ::std::atomic<bool> run_{true};

  void Run();
  void RunIteration();

  pid_t photos_capture_pid_;
  pid_t photos_download_pid_;

  double take_photos_triggered_;

  bool photos_available_to_download_;
};

}  // namespace lib

#endif  // LIB_DSLR_INTERFACE_DSLR_INTERFACE_H_
