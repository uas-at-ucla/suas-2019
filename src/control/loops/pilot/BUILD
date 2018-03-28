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
    '//lib/physics_structs:physics_structs',
    '//lib/mission_manager:mission_manager',
    '//src/control/ground_communicator:mission_commands_cpp',
    "@cppzmq//:cppzmq",
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
