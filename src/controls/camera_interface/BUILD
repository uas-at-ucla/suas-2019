cc_binary(
  name = 'camera_interface',
  visibility = ['//visibility:public'],
  srcs = [
    'camera_interface_main.cc'
  ],
  deps = [
    ':camera_interface_lib',
  ],
)

cc_library(
  name = 'camera_interface_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'camera_interface.cc'
  ],
  hdrs = [
    'camera_interface.h',
  ],
  deps = [
    '//src/controls:constants',
    '//src/controls:messages_cpp',
    '//lib/ros:ros',
    "@sensor_msgs//:sensor_msgs",
  ],
)
