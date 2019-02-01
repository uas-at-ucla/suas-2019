cc_library(
  name = 'autopilot_interface',
  visibility = ['//visibility:public'],
  srcs = [
    'autopilot_interface.cc',
  ],
  hdrs = [
    'autopilot_interface.h',
  ],
  deps = [
    '//src/controls:messages_cpp',
    '//src/controls/io/autopilot_interface/ros_publisher:ros_publisher',

    '//lib/ros:ros',
    '//lib/mavconn_udp:mavconn_udp'
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
  ],
)
