package(default_visibility = ['//visibility:public'])

load('//aos/build_queues:queues.bzl', 'queue_library')

queue_library(
  name = 'flight_loop_queue',
  srcs = [
    'flight_loop.q',
  ],
)

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

cc_binary(
  name = 'flight_loop',
  visibility = ['//visibility:public'],
  srcs = [
    'flight_loop_main.cc'
  ],
  deps = [
    ":flight_loop_lib"
  ],
)

cc_binary(
  # TODO: Use bazel test.
  # Requires bazel build @PX4_sitl//:jmavsim and bazel build //src/... to be run
  # beforehand.

  name = 'flight_loop_lib_test',
  srcs = [
    'flight_loop_lib_test.cc',
  ],
  deps = [
    ':flight_loop_lib',
    '@gtest//:googletest',
  ],
)

cc_library(
  name = 'flight_loop_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'flight_loop.cc'
  ],
  hdrs = [
    'flight_loop.h',
  ],
  deps = [
    ':flight_loop_queue',
    '//src/control/loops/pilot:pilot',
    '//lib/physics_structs:physics_structs',
    '//aos/common/util:phased_loop',
    '//aos/linux_code:init',
    '//aos/common:time',
    "@cppzmq//:cppzmq",
  ],
)
