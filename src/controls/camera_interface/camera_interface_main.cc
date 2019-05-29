#include "camera_interface.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_camera_interface");
  ::ros::start();

  ::src::controls::io::camera_interface::CameraInterface camera_interface;

  ::ros::spin();
}
