package(default_visibility = ['//visibility:public'])

proto_library(
  name = "timeline_grammar",
  srcs = [
    "timeline_grammar.proto",
    
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
    "//lib/mission_manager:mission_commands",
  ],
)

cc_proto_library(
  name = "timeline_grammar_cc",
  deps = [
    ":timeline_grammar",
  ],
)
