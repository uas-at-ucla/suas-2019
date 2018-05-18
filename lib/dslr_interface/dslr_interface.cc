#include "semaphore.h"

namespace lib {

DSLRInterface::DSLRInterface()
    : take_photos_triggered_(0),
      thread_(&MissionMessageQueueReceiver::Run, this) {}

void DSLRInterface::Quit() { KillDSLRAndWait(); }

void DSLRInterface::Run() {
  while (run_) {
    RunIteration();
  }
}

void DSLRInterface::RunIteration() {
  switch (state_) {
    case STANDBY:
      break;

    case CONTINUOUSLY_TAKE_PHOTOS:

      break;

    case START_DOWNLOAD_PHOTOS:

      state = DOWNLOAD_PHOTOS;
      break;

    case DOWNLOAD_PHOTOS:
      if (kill(camera_trigger_pid_, 0)) {
        // Photos downloader finished.
        state_ = STANDBY;
      }

      break;

    case STOP_DOWNLOADING_PHOTOS:
      // Kill photos downloader.
      kill(-1 * photos_download_pid_, SIGINT);

    case WAIT_FOR_DOWNLOADING_PHOTOS_EXIT:
      if (kill(photos_download_pid_, 0)) {
        // Photos downloader successfully killed.
        state_ = STANDBY;
      }

      break;
  }
}

void DSLRInterface::TakePhotos() {
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  take_photos_triggered_ = current_time;
}

}  // namespace lib
