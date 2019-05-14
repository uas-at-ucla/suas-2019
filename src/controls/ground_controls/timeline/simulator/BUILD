cc_library(
  name = 'simulator',
  visibility = ['//visibility:public'],
  srcs = [
    'simulator.cc'
  ],
  hdrs = [
    'simulator.h'
  ],
  deps = [
    '@eigen',
  ],
)

cc_binary(
  name = 'simulator_test',
  srcs = [
    'simulator_test.cc',
  ],
  deps = [
    ':simulator',
    '@gtest//:gtest_main',
  ],
)
