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
    ':flight_state_machine',
    '//src/controls:constants',
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

cc_library(
  name = 'flight_state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'flight_state_machine.cc',
  ],
  hdrs = [
    'flight_state_machine.h',
  ],
  deps = [
    ':mission_state_machine',

    '//src/controls:messages_cpp',
    '//src/controls/ground_controls/timeline:timeline_grammar_cc',
    '//src/controls:constants',

    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/phased_loop:phased_loop',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
  ],
)

cc_library(
  name = 'mission_state_machine',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_state_machine.cc',
  ],
  hdrs = [
    'mission_state_machine.h',
  ],
  deps = [
    '//src/controls:messages_cpp',

    '//src/controls/ground_controls/timeline:timeline_grammar_cc',
    '//lib/proto_comms:proto_comms',
    '//lib/physics_structs:physics_structs',
    '//lib/phased_loop:phased_loop',
    '//lib/ros:ros',
    '//lib/alarm:alarm',
  ],
)
