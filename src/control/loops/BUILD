package(default_visibility = ['//visibility:public'])

load('//aos/build_queues:queues.bzl', 'queue_library')

queue_library(
  name = 'flight_loop_queue',
  srcs = [
    'flight_loop.q',
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
    '//src/control:messages_cpp',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/logger:log_sender',
    '//lib/alarm:alarm',
    '//aos/common/util:phased_loop',
    '//aos/linux_code:init',
    '//aos/common:time',
    "@cppzmq//:cppzmq",
  ],
)
