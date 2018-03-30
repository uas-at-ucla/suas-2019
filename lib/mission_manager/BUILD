package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'mission_manager',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_manager.cc',
  ],
  hdrs = [
    'mission_manager.h',
  ],
)

cc_test(
  name = 'mission_manager_test',
  srcs = [
    'mission_manager_test.cc',
  ],
  deps = [
    ':mission_manager',
    '@gtest//:googletest_main',
  ],
)
