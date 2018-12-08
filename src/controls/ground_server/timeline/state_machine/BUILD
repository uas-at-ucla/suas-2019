cc_library(
  name = 'state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'state_machine.cc'
  ],
  hdrs = [
    'state.hh',
    'branching_state.hh',
    'state_machine.hh'
  ],
  deps = [
  ],
)

cc_binary(
  name = 'state_machine_test',
  srcs = [
    'state_machine_test.cc',
  ],
  deps = [
    ':state_machine',
    '@gtest//:gtest_main',
  ],
)
