package(default_visibility = ['//visibility:public'])

proto_library(
  name = "mission_commands",
  srcs = [
    "mission_commands.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "mission_commands_cpp",
  deps = [
    ":mission_commands",
  ],
)

cc_library(
  name = 'mission_manager',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_manager.cc',
  ],
  hdrs = [
    'mission_manager.h',
  ],
  deps = [
    ':mission_commands_cpp',
    '//lib/rrt_avoidance:rrt_avoidance',
    '//lib/semaphore:semaphore',
  ],
)

cc_test(
  name = 'mission_manager_test',
  srcs = [
    'mission_manager_test.cc',
  ],
  deps = [
    ':mission_manager',
    '@gtest//:gtest_main',
  ],
)
