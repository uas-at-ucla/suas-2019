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
    EXITED = 1,
    START_CONTINUOUS_PHOTO_CAPTURE = 2,
    CONTINUOUS_PHOTO_CAPTURE = 3,
    STOP_CONTINUOUS_PHOTO_CAPTURE = 4,
    WAIT_FOR_PHOTOS_CAPTURE_EXIT = 5,
    START_DOWNLOAD_PHOTOS = 6,
    DOWNLOAD_PHOTOS = 7,
    STOP_DOWNLOADING_PHOTOS = 8,
    WAIT_FOR_DOWNLOADING_PHOTOS_EXIT = 9
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
