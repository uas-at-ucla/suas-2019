#include "camera_interface.h"

namespace src {
namespace controls {
namespace camera_interface {

CameraInterface::CameraInterface() :
    tag_file_(kCameraInterfaceTagFile,
              ::std::ofstream::out | ::std::ofstream::app),
    last_tag_write_(0),
    sensors_subscriber_(
        ros_node_handle_.subscribe(kRosSensorsTopic, kRosMessageQueueSize,
                                   &CameraInterface::SensorsReceived, this)) {
  tag_file_.precision(17);
}

void CameraInterface::Quit(int signal) {
  (void)signal;

  tag_file_.close();
}

void CameraInterface::SensorsReceived(::src::controls::Sensors sensors) {
  // Limit rate at which we write sensors to file.
  double current_time = ::ros::Time::now().toSec();
  if (current_time < last_tag_write_ + 1.0 / kCameraTagWriteHz) {
    return;
  }

  last_tag_write_ = current_time;

  // Write out the tag data.
  WriteTag(sensors.latitude(), sensors.longitude(), sensors.relative_altitude(),
           sensors.heading());
}

void CameraInterface::WriteTag(double latitude, double longitude,
                               double altitude, double heading) {
  // Write coordinates to the log file
  double timestamp = ::std::chrono::duration_cast<::std::chrono::milliseconds>(
                         ::std::chrono::system_clock::now().time_since_epoch())
                         .count() /
                     1000.0;

  tag_file_ << timestamp << "," << latitude << "," << longitude << ","
            << altitude << "," << heading << ::std::endl;

  tag_file_.flush();
}

} // namespace camera_interface
} // namespace controls
} // namespace src
