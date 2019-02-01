cc_library(
  name = 'ros_publisher',
  visibility = ['//visibility:public'],
  srcs = [
    'ros_publisher.cc',
  ],
  hdrs = [
    'ros_publisher.h',
  ],
  deps = [
    '//src/controls:messages_cpp',

    '//lib/ros:ros',
    '//lib/mavconn_udp:mavconn_udp'
  ],
)
