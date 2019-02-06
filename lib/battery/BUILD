package(default_visibility = ['//visibility:public'])

cc_test(
  name = 'battery_test',
  srcs = [
    'battery_test.cc',
  ],
  deps = [
    ':battery',
    '@gtest//:gtest_main',
  ],
)

cc_library(
  name = 'battery',
  visibility = ['//visibility:public'],
  srcs = [
    'battery.cc',
  ],
  hdrs = [
    'battery.h',
  ],
  deps = [
    '//lib/physics_structs:physics_structs',
    '@eigen',
  ],
)
