package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'executor',
  visibility = ['//visibility:public'],
  srcs = [
    'executor.cc'
  ],
  hdrs = [
    'executor.h'
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
  name = "executor_test",
  srcs = ["executor_test.cc"],
  deps = [
    ":executor",
    "@gtest//:gtest_main"
  ],
)
