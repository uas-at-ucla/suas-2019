#include "camera_interface.h"

#include <string>
#include <iostream>
#include <fstream>
#include <chrono>

void add_tag(double lat, double lng, double heading, double altitude);

static ::src::controls::io::camera_interface::CameraInterface
    *camera_interface = nullptr;

extern "C" void signal_handler(int signum) {
  ::std::cout << "GOT SIGNAL!" << ::std::endl;
  if (camera_interface != nullptr) {
    camera_interface->Quit(signum);
  }

  ::ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  // Signal handlers
  signal(SIGQUIT, signal_handler);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGHUP, signal_handler);

  ::ros::init(argc, argv, "uasatucla_camera_interface");
  ::ros::start();

  camera_interface =
      new ::src::controls::io::camera_interface::CameraInterface();

  ::ros::spin();
}

void add_tag(double lat, double lng, double heading, double altitude) {
  ::std::string fname = "/home/pi/pictures/image_labels.csv";
  // ^ change directory for testing purposes
  ::std::ofstream tagfile (fname, ::std::ofstream::out | ::std::ofstream::app);
  tagfile.precision(17); // should be max precision of double

  // Write coordinates to the log file
  long timestamp = 
      ::std::chrono::duration_cast< ::std::chrono::milliseconds > (
          ::std::chrono::system_clock::now().time_since_epoch()   )
          .count();
  tagfile << timestamp << "," << lat << "," << lng << "," <<
             heading << "," << altitude << ::std::endl;
  tagfile.close();
}

