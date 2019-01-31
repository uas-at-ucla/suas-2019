cc_library(
  name = 'mission_state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_state_machine.cc',

    'UGVRelease_state.cc',
    'translate_state.cc',
    'get_next_cmd_state.cc',
    'unknown_state.cc',
  ],
  hdrs = [
    'mission_state_machine.h',
  ],
  deps = [
    '//src/controls:messages_cpp',

    '//src/controls/ground_server/timeline/executor:executor',
    '//lib/logger:log_writer_lib',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/logger:log_sender',
    '//lib/phased_loop:phased_loop',
    '//lib/alarm:alarm',
  ],
)
