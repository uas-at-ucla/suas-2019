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
    '@gtest//:gtest',
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
    '//src/controls/ground_controls/timeline/executor:executor',
    '//src/controls/flight_loop/state_machine:state_machine',
    '//src/controls:messages_cpp',
    '//src/controls/io:io_lib',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/phased_loop:phased_loop',
    '//lib/alarm:alarm',
    '//lib/ros:ros',
    "@cppzmq//:cppzmq",
    '@boost//:algorithm',
    "@mavros_msgs//:mavros_msgs",
    "@sensor_msgs//:sensor_msgs",
  ],
)