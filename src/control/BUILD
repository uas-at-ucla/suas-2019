package(default_visibility = ['//visibility:public'])

proto_library(
  name = "messages",
  srcs = [
    "messages.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "messages_cpp",
  deps = [
      ":messages",
  ],
)
