cc_library(
  name = 'ros_to_proto',
  visibility = ['//visibility:public'],
  srcs = [
    'ros_to_proto.cc'
  ],
  hdrs = [
    'ros_to_proto.h'
  ],
  deps = [
    '//src/controls:messages_cpp',
    '//lib/phased_loop:phased_loop',
    '//lib/ros:ros',
    "@mavros_msgs//:mavros_msgs",
    "@sensor_msgs//:sensor_msgs",
  ],
)
