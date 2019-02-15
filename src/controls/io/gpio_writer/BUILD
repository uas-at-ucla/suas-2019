cc_binary(
  name = 'gpio_writer',
  visibility = ['//visibility:public'],
  srcs = [
    'gpio_writer_main.cc'
  ],
  deps = [
    ":gpio_writer_lib"
  ],
)

cc_library(
  name = 'gpio_writer_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'gpio_writer.cc'
  ],
  hdrs = [
    'gpio_writer.h'
  ],
  deps = [
    '//src/controls:messages_cpp',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
    "@mavros_msgs//:mavros_msgs",
  ],
)
