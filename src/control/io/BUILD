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
    ':loop_output_handler',
    ':loop_input_handler',
    '//src/control/io/autopilot_interface:autopilot_interface',
    '//src/control:messages_cpp',
    '//lib/logger:log_sender',
    '//lib/dslr_interface:dslr_interface',
    '//lib/proto_comms:proto_comms',
    '//lib/trigger:trigger',
    "@WiringPi//:WiringPi",
    "@pigpio//:pigpio",
  ],
)

cc_library(
  name = 'loop_output_handler',
  visibility = ['//visibility:public'],
  srcs = [
    'loop_output_handler.cc'
  ],
  hdrs = [
    'loop_output_handler.h'
  ],
  deps = [
    '//lib/phased_loop:phased_loop',
  ],
)

cc_library(
  name = 'loop_input_handler',
  visibility = ['//visibility:public'],
  srcs = [
    'loop_input_handler.cc'
  ],
  hdrs = [
    'loop_input_handler.h'
  ],
  deps = [
    '//lib/phased_loop:phased_loop',
  ],
)
