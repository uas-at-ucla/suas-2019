cc_library(
  name = 'drone_state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'drone_state_machine.cc',
    'drone_state.cc',
    'states/nothing.cc',
    'states/sleep.cc',
    'states/translate.cc',
    'states/trigger_alarm.cc',
    'states/trigger_bomb_drop.cc'
  ],
  hdrs = [
    'drone_state_machine.hh',
    'drone_state.hh',
    'states.hh',
    'states/nothing.hh',
    'states/sleep.hh',
    'states/translate.hh',
    'states/trigger_alarm.hh',
    'states/trigger_bomb_drop.hh'
  ],
  deps = [
    '//src/controls/ground_server/timeline/state_machine',
    '//src/controls:messages_cpp',
  ],
)
