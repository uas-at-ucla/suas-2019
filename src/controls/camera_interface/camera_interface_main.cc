#include "camera_interface.h"

::src::controls::camera_interface::CameraInterface *camera_interface;
extern "C" void signal_handler(int signum) {
  (void)signum;

  if (camera_interface != nullptr) {
    camera_interface->Quit(signum);
  }

  ::ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  // Register ROS process.
  ::ros::init(argc, argv, "uasatucla_camera_interface",
              ::ros::init_options::NoSigintHandler);

  // Create signal handlers.
  signal(SIGQUIT, signal_handler);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGHUP, signal_handler);

  // Start ROS threads.
  ::ros::start();

  // Initialize CameraInterface.
  camera_interface = new ::src::controls::camera_interface::CameraInterface();

  // Spin forever.
  ::ros::spin();

  return 0;
}
