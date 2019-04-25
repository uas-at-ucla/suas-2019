cc_binary(
  name = 'ground_controls',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_controls_main.cc'
  ],
  deps = [
    # ":ground_controls_lib",


    '//src/controls:messages_cpp',
    '//lib/mission_message_queue:mission_message_queue',
    '//lib/serial_comms:serial_comms_bridge',
    '//lib/phased_loop:phased_loop',
    '//lib/proto_comms:proto_comms',
    '//lib/base64_tools:base64_tools',
    '//lib/ros:ros',
    '//lib/serial_device:serial_device',
    "@mavros_msgs//:mavros_msgs",
    "@sensor_msgs//:sensor_msgs",
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)

# cc_library(
#   name = 'ground_controls_lib',
#   visibility = ['//visibility:public'],
#   srcs = [
#     'ground_controls.cc'
#   ],
#   hdrs = [
#     'ground_controls.h'
#   ],
#   deps = [
#     '//src/controls/loops:flight_loop_lib',
#     '//src/controls:messages_cpp',
#     '//lib/mission_message_queue:mission_message_queue',
#     '//lib/serial_comms:serial_comms_bridge',
#     '//lib/phased_loop:phased_loop',
#     '//lib/proto_comms:proto_comms',
#     '//lib/base64_tools:base64_tools',
#     '//lib/ros:ros',
#     "@mavros_msgs//:mavros_msgs",
#     "@sensor_msgs//:sensor_msgs",
#     "@socketio_client_cpp//:socketio_client_cpp",
#     "@cppzmq//:cppzmq",
#   ],
# )
