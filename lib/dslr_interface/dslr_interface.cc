#include "dslr_interface.h"

namespace lib {
namespace {

#ifdef UAS_AT_UCLA_DEPLOYMENT
const char *kTakePhotosBashScriptLocation =
    "/home/pi/drone_code_deploy/scripts/take_photos_continuously.sh";
const char *kDownloadPhotosBashScriptLocation =
    "/home/pi/drone_code_deploy/scripts/download_photos.sh";
#else
const char *kTakePhotosBashScriptLocation =
    "./lib/scripts/take_photos_continuously.sh";
const char *kDownloadPhotosBashScriptLocation =
    "./lib/scripts/download_photos.sh";
#endif

} // namespace

DSLRInterface::DSLRInterface() :
    state_(STANDBY),
    thread_(&DSLRInterface::Run, this),
    phased_loop_(25),
    take_photos_triggered_(0),
    photos_available_to_download_(true) {}

void DSLRInterface::Quit() {
  exiting_ = true;
  thread_.join();

  // Fork a process for downloading photos from the DSLR.
  int unmount_dslr_pid = fork();
  if (!unmount_dslr_pid) {
    setsid();
    execl("/bin/fusermount", "/tmp/drone_code_dslr_mounted", NULL);
    exit(0);
  }

  // Fork a process for downloading photos from the DSLR.
  int kill_gphotofs_pid = fork();
  if (!kill_gphotofs_pid) {
    setsid();
    execl("/usr/bin/killall", "killall", "gphotofs", NULL);
    exit(0);
  }

  exit(0);
}

void DSLRInterface::Run() {
  while (run_) {
    RunIteration();
  }
}

void DSLRInterface::RunIteration() {
  phased_loop_.SleepUntilNext();

  State next_state = state_;

  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  int pid_stat;

  switch (state_) {
    case STANDBY:
      if (exiting_) {
        next_state = EXITED;
      } else if (take_photos_triggered_ + 0.25 > current_time) {
        next_state = START_CONTINUOUS_PHOTO_CAPTURE;
      } else if (photos_available_to_download_) {
        next_state = START_DOWNLOAD_PHOTOS;
      }

      break;

    case EXITED:
      run_ = false;
      break;

    case START_CONTINUOUS_PHOTO_CAPTURE:
      photos_available_to_download_ = true;

      photos_capture_pid_ = fork();

      if (!photos_capture_pid_) {
        setsid();
        execl("/bin/sh", "sh", kTakePhotosBashScriptLocation, NULL);
        exit(0);
      }

      next_state = CONTINUOUS_PHOTO_CAPTURE;
      break;

    case CONTINUOUS_PHOTO_CAPTURE:
      if (exiting_) {
        next_state = STOP_CONTINUOUS_PHOTO_CAPTURE;
      } else if (waitpid(photos_capture_pid_, &pid_stat, WNOHANG)) {
        // Photos capture finished successfully.
        next_state = STANDBY;
      } else if (take_photos_triggered_ + 0.25 <= current_time) {
        // Camera doesn't need to take photos anymore, so stop capturing.
        next_state = STOP_CONTINUOUS_PHOTO_CAPTURE;
      }

      break;

    case STOP_CONTINUOUS_PHOTO_CAPTURE:
      // Interrupt photos capture.
      kill(-1 * photos_capture_pid_, SIGINT);
      next_state = WAIT_FOR_PHOTOS_CAPTURE_EXIT;

      break;

    case WAIT_FOR_PHOTOS_CAPTURE_EXIT:
      if (waitpid(photos_capture_pid_, &pid_stat, WNOHANG)) {
        // Photos capture successfully killed.
        next_state = STANDBY;
      } else {
        // kill(-1 * photos_capture_pid_, SIGINT);
      }

      break;

    case START_DOWNLOAD_PHOTOS:
      photos_download_pid_ = fork();

      // Fork a process for downloading photos from the DSLR.
      if (!photos_download_pid_) {
        setsid();
        execl("/bin/sh", "sh", kDownloadPhotosBashScriptLocation, NULL);
        exit(0);
      }

      next_state = DOWNLOAD_PHOTOS;
      break;

    case DOWNLOAD_PHOTOS:
      if (exiting_) {
        next_state = STOP_DOWNLOADING_PHOTOS;
      } else if (waitpid(photos_download_pid_, &pid_stat, WNOHANG)) {
        // Photos downloader finished successfully.
        photos_available_to_download_ = false;
        next_state = STANDBY;
      } else if (take_photos_triggered_ + 0.25 > current_time) {
        // Camera needs to take photos, so stop downloading.
        next_state = STOP_DOWNLOADING_PHOTOS;
      }

      break;

    case STOP_DOWNLOADING_PHOTOS:
      // Interrupt photos downloader.
      kill(-1 * photos_download_pid_, SIGINT);
      next_state = WAIT_FOR_DOWNLOADING_PHOTOS_EXIT;
      break;

    case WAIT_FOR_DOWNLOADING_PHOTOS_EXIT:
      if (waitpid(photos_download_pid_, &pid_stat, WNOHANG)) {
        // Photos downloader successfully killed.
        next_state = STANDBY;
      }

      kill(photos_download_pid_, SIGINT);

      break;
  }

  state_ = next_state;
}

void DSLRInterface::TakePhotos() {
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  take_photos_triggered_ = current_time;
}

} // namespace lib
