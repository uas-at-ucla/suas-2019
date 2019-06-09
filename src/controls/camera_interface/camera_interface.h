#pragma once

#include <fstream>
#include <iostream>

#include <ros/console.h>
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>

#include "src/controls/constants.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace camera_interface {
namespace {
static const ::std::string kCameraInterfaceTagFile = "/home/uas/image_tags.csv";
static constexpr double kCameraTagWriteHz = 4;
} // namespace

class CameraInterface {
 public:
  CameraInterface();
  void Quit(int signal);

 private:
  void SensorsReceived(::src::controls::Sensors sensors);
  void WriteTag(double latitude, double longitude, double altitude,
                double heading);

  ::std::ofstream tag_file_;

  double last_tag_write_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;
};

} // namespace camera_interface
} // namespace controls
} // namespace src
