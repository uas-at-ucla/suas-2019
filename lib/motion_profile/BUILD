package(default_visibility = ['//visibility:public'])

cc_test(
  name = 'motion_profile_test',
  srcs = [
    'motion_profile_test.cc',
  ],
  deps = [
    ':motion_profile',
    '@gtest//:gtest_main',
  ],
)

cc_library(
  name = 'motion_profile',
  visibility = ['//visibility:public'],
  srcs = [
    'motion_profile.cc',
  ],
  hdrs = [
    'motion_profile.h',
  ],
  deps = [
    '//lib/physics_structs:physics_structs',
    '@eigen',
  ],
)
