cc_binary(
  name = 'ground_server',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_server_main.cc'
  ],
  deps = [
    ":ground_server_lib"
  ],
)

cc_library(
  name = 'ground_server_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_server.cc'
  ],
  hdrs = [
    'ground_server.h'
  ],
  deps = [
    '//src/controls:messages_cpp',
    '//lib/ros:ros',
    "@socketio_client_cpp//:socketio_client_cpp",
  ],
)
