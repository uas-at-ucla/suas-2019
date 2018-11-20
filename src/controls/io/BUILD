cc_binary(
  name = 'io',
  visibility = ['//visibility:public'],
  srcs = [
    'io_main.cc'
  ],
  deps = [
    ":io_lib"
  ],
  copts = [
    "-g",
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
    ':loop_handler',
    '//src/controls/io/autopilot_interface:autopilot_interface',
    '//src/controls:messages_cpp',
    '//lib/logger:log_sender',
    '//lib/dslr_interface:dslr_interface',
    '//lib/proto_comms:proto_comms',
    '//lib/trigger:trigger',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
  ],
)

cc_library(
  name = 'loop_handler',
  visibility = ['//visibility:public'],
  srcs = [
    'loop_handler.cc'
  ],
  hdrs = [
    'loop_handler.h'
  ],
  deps = [
    '//lib/phased_loop:phased_loop',
  ],
)
