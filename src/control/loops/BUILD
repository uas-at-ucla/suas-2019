package(default_visibility = ['//visibility:public'])

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
    '//src/control/loops/pilot:pilot',
    '//src/control:messages_cpp',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/logger:log_sender',
    '//lib/phased_loop:phased_loop',
    '//lib/alarm:alarm',
    "@cppzmq//:cppzmq",
  ],
)
