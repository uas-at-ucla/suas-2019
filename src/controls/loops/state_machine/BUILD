cc_library(
  name = 'state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'state_machine.cc',

    'standby_state.cc',
    'arming_state.cc',
    'armed_wait_for_spinup_state.cc',
    'armed_state.cc',
    'taking_off_state.cc',
    'taken_off_state.cc',
    'safety_pilot_control_state.cc',
    'mission_state.cc',
    'landing_state.cc',
    'failsafe_state.cc',
    'flight_termination_state.cc',
    'unknown_state.cc',
  ],
  hdrs = [
    'state_machine.h',
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
