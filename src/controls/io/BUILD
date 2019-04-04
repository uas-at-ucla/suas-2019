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
    '//src/controls:messages_cpp',
    '//src/controls/io/io/led_strip:led_strip',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
    '//lib/phased_loop:phased_loop',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
    "@mavros_msgs//:mavros_msgs",
    "@sensor_msgs//:sensor_msgs",
  ],
)
