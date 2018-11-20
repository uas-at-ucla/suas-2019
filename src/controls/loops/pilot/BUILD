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
    '//lib/mission_message_queue:mission_message_queue',
    '//lib/pid:pid',
    '//lib/motion_profile:motion_profile',
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
