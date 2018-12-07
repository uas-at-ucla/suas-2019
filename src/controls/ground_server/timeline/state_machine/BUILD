cc_library(
  name = 'state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'state_machine.cc'
  ],
  hdrs = [
    'state_machine.h'
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
