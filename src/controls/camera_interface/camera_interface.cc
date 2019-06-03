#include "camera_interface.h"

namespace src {
namespace controls {
namespace io {
namespace camera_interface {

CameraInterface::CameraInterface() :
    ros_node_handle_(),
    take_photo_subscriber_(ros_node_handle_.subscribe(
        kRosTakePhotoTopic, kRosMessageQueueSize,
        &CameraInterface::ImageCaptureRequested, this)),
    photo_thread_(&CameraInterface::PhotoThread, this),
    photo_phased_loop_(kWriterPhasedLoopFrequency) {}

void CameraInterface::ImageCaptureRequested(const std_msgs::String &ret) {
  (void)ret;
  dslr_.TakePhotos();
}

void CameraInterface::PhotoThread() {
  while (running_ && ::ros::ok()) {
    dslr_.RunIteration();
    ::ros::spinOnce();
    photo_phased_loop_.sleep();
  }
}

void CameraInterface::Quit(int signal) {
  (void)signal;
  running_ = false;
  photo_thread_.join();
}

} // namespace camera_interface
} // namespace io
} // namespace controls
} // namespace src
