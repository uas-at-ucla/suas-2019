cc_binary(
  name = 'io',
  visibility = ['//visibility:public'],
  srcs = [
    'io_main.cc'
  ],
  deps = [
    ":io_lib"
  ],
)

cc_library(
  name = 'io_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'io.cc'
  ],
  hdrs = [
    'io.h'
  ],
  deps = [
    '//src/controls:constants',
    '//src/controls:messages_cpp',
    '//src/controls/ground_controls/timeline:timeline_grammar_cc',
    '//src/controls/io/led_strip:led_strip',
    '//src/controls/io/ros_to_proto:ros_to_proto',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
    '//lib/deployment:deployment',
    '//lib/phased_loop:phased_loop',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
    "@mavros_msgs//:mavros_msgs",
    "@sensor_msgs//:sensor_msgs",
    "@GeographicLib//:GeographicLib",
  ],
)
