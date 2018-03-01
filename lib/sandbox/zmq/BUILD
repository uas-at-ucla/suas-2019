cc_binary(
  name = "server",
  srcs = ["server.cc"],
  deps = [
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
    ":mission_commands_cpp",
  ],
)

cc_binary(
  name = "client",
  srcs = ["client.cc"],
  deps = [
    ":mission_commands_cpp",
    "@cppzmq//:cppzmq"
  ],
)

proto_library(
  name = "mission_commands",
  srcs = [
    "mission_commands.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "mission_commands_cpp",
  deps = [
    ":mission_commands",
  ],
)
