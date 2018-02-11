cc_library(
  name = "mavlink_control_lib",
  visibility = ["//visibility:public"],
  hdrs = glob([
    "**/*.h"
  ]),
  deps = [
    "@mavlink_v2//:mavlink_v2"
  ]
)

cc_binary(
  name = "mavlink_control",
  visibility = ["//visibility:public"],
  srcs = glob([
    "mavlink_control.cc",
    "serial_port.cc",
    "autopilot_interface.cc",
  ]),
  deps = [
    ":mavlink_control_lib"
  ],
  copts = [
    "-Wno-unused-parameter",
    "-Wno-missing-field-initializers",
    "-Wno-format",
  ],
  linkopts = [
    "-lpthread"
  ]
)
