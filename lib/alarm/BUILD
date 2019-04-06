package(default_visibility = ['//visibility:public'])

cc_test(
  name = 'alarm_test',
  srcs = [
    'alarm_test.cc',
  ],
  deps = [
    ':alarm',
    '@gtest//:gtest_main',
  ],
)

cc_library(
  name = 'alarm',
  visibility = ['//visibility:public'],
  srcs = [
    'alarm.cc',
  ],
  hdrs = [
    'alarm.h',
  ],
)
