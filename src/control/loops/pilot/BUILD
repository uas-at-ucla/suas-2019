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
    '//src/control/loops:flight_loop_queue',
    '//lib/physics_structs:physics_structs',
    '//lib/mission_message_queue:mission_message_queue',
    '//aos/linux_code:init',
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
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
