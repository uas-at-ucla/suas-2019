cc_binary(
  name = 'actuators_writer',
  visibility = ['//visibility:public'],
  srcs = [
    'actuators_writer_main.cc'
  ],
  deps = [
    ":actuators_writer_lib"
  ],
)

cc_library(
  name = 'actuators_writer_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'actuators_writer.cc'
  ],
  hdrs = [
    'actuators_writer.h'
  ],
  deps = [
    '//src/controls:messages_cpp',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
  ],
)
