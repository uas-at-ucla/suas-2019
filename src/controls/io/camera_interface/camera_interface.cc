#include "camera_interface.h"

namespace src {
namespace controls {
namespace io {
namespace camera_interface {

CameraInterface::CameraInterface() :
    take_photo_subscriber_(ros_node_handle_.subscribe(
        "/uasatucla/camera/take_photo", 1,
        &CameraInterface::ImageCaptureRequested, this)) {}

void CameraInterface::ImageCaptureRequested(const std_msgs::String &ret) {
  (void)ret;
}

} // namespace camera_interface
} // namespace io
} // namespace controls
} // namespace src
