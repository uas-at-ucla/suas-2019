cc_binary(
  name = 'ground_communicator',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_communicator_main.cc'
  ],
  deps = [
    ":ground_communicator_lib"
  ],
)

cc_library(
  name = 'ground_communicator_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_communicator.cc'
  ],
  hdrs = [
    'ground_communicator.h'
  ],
  deps = [
    '//src/controls:constants',
    '//src/controls:messages_cpp',
    '//src/controls/io:io_lib',
    '//src/controls/ground_controls:ground_controls_lib',
    '//lib/ros:ros',
    '//lib/serial_device:serial_device',
    '//lib/proto_comms:proto_comms',
  ],
)
