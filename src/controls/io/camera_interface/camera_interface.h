#pragma once

#include <functional>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace src {
namespace controls {
namespace io {
namespace camera_interface {

class CameraInterface {
 public:
  CameraInterface();
  void ImageCaptureRequested(const std_msgs::String &ret);

 private:
  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber take_photo_subscriber_;
};

} // namespace camera_interface
} // namespace io
} // namespace controls
} // namespace src
