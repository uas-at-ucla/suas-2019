cc_binary(
  name = 'io_sim',
  visibility = ['//visibility:public'],
  srcs = [
    'io_sim_main.cc'
  ],
  deps = [
    ":io_sim_lib"
  ],
  copts = [
    "-g",
  ],
)

cc_library(
  name = 'io_sim_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'io_sim.cc'
  ],
  hdrs = [
    'io_sim.h'
  ],
  deps = [
    '//src/controls/io:loop_handler',
    '//src/controls:messages_cpp',
    '//lib/logger:log_sender',
    '//lib/proto_comms:proto_comms',
    '//lib/trigger:trigger',
  ],
)
