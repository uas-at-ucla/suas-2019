cc_library(
  name = 'drone_plant',
  visibility = ['//visibility:public'],
  srcs = [
    'drone_plant.cc'
  ],
  hdrs = [
    'drone_plant.h'
  ],
  deps = [
    '@eigen',
    '//lib/motion_profile:motion_profile',
  ],
)

cc_binary(
  name = 'drone_plant_test',
  srcs = [
    'drone_plant_test.cc',
  ],
  deps = [
    ':drone_plant',
    '@gtest//:googletest_main',
  ],
)
