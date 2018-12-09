cc_library(
  name = 'drone_state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'drone_state_machine.cc',
    'drone_state.cc',
    'states/nothing.cc',
    'states/sleep.cc'
  ],
  hdrs = [
    'drone_state_machine.hh',
    'drone_state.hh',
    'states/nothing.hh',
    'states/sleep.hh'
  ],
  deps = [
    '//src/controls/ground_server/timeline/state_machine'
  ],
)
