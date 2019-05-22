cc_library(
  name = 'state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'state_machine.cc',
  ],
  hdrs = [
    'state_machine.h',
  ],
  deps = [
    '//src/controls:messages_cpp',

    '//src/controls/ground_controls/timeline/executor:executor',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/phased_loop:phased_loop',
    '//lib/alarm:alarm',
  ],
)
