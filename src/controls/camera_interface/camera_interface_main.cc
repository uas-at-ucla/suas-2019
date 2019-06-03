#include "camera_interface.h"

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
