package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'pilot',
  visibility = ['//visibility:public'],
  srcs = [
    'pilot.cc'
  ],
  hdrs = [
    'pilot.h'
  ],
  deps = [
    '//lib/physics_structs:physics_structs'
  ],
)

cc_test(
  name = "pilot_test",
  srcs = ["pilot_test.cc"],
  deps = [
    ":pilot",
    "@gtest//:googletest_main"
  ],
)
