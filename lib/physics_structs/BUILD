package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'physics_structs',
  visibility = ['//visibility:public'],
  srcs = [
    'physics_structs.cc'
  ],
  hdrs = [
    'physics_structs.h'
  ],
)

cc_test(
  name = 'physics_structs_test',
  srcs = [
    'physics_structs_test.cc',
  ],
  deps = [
    ':physics_structs',
    '@gtest//:gtest_main',
  ],
)

