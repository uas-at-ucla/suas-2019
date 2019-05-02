cc_binary(
  name = 'ground_controls',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_controls_main.cc'
  ],
  deps = [
    ":ground_controls_lib",
  ],
)

cc_library(
  name = 'ground_controls_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_controls.cc'
  ],
  hdrs = [
    'ground_controls.h'
  ],
  deps = [
    '//src/controls/loops:flight_loop_lib',
    '//src/controls:messages_cpp',
    '//src/controls/io:io_lib',
    '//lib/mission_message_queue:mission_message_queue',
    '//lib/serial_comms:serial_comms_bridge',
    '//lib/serial_device:serial_device',
    '//lib/phased_loop:phased_loop',
    '//lib/proto_comms:proto_comms',
    '//lib/base64_tools:base64_tools',
    '//lib/ros:ros',
    "@socketio_client_cpp//:socketio_client_cpp",

    '//src/controls/ground_server/timeline/ground2drone_visitor:ground2drone_visitor',
    '//src/controls/ground_server/timeline:timeline_grammar_cc',
  ],
)