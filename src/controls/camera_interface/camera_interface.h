#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "lib/dslr_interface/dslr_interface.h"
#include "ros/ros.h"
#include "src/controls/constants.h"
#include "std_msgs/String.h"

namespace src {
namespace controls {
namespace io {
namespace camera_interface {

class CameraInterface {
 public:
  CameraInterface();
  void Quit(int signal);
  void ImageCaptureRequested(const std_msgs::String &ret);

 private:
  void PhotoThread();

  ::std::atomic<bool> running_{true};

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber take_photo_subscriber_;
  ::lib::DSLRInterface dslr_;

  ::std::thread photo_thread_;
  ::ros::Rate photo_phased_loop_;
};

} // namespace camera_interface
} // namespace io
} // namespace controls
} // namespace src
