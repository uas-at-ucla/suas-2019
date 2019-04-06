cc_library(
  name = 'mission_message_queue',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_message_queue.cc',
  ],
  hdrs = [
    'mission_message_queue.h',
  ],
  deps = [
    "//lib/mission_manager:mission_commands_cpp",
    "//lib/mission_manager:mission_manager",
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
  ],
)

cc_test(
  name = 'mission_message_queue_test',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_message_queue_test.cc',
  ],
  deps = [
    ":mission_message_queue",
    "@gtest//:gtest_main",
  ],
)
