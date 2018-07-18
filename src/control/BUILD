package(default_visibility = ['//visibility:public'])

proto_library(
  name = "control_messages",
  srcs = [
    "control_messages.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "control_messages_cpp",
  deps = [
      ":control_messages",
  ],
)
