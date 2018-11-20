package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'timeline',
  visibility = ['//visibility:public'],
  srcs = [
    'timeline.cc'
  ],
  hdrs = [
    'timeline.h'
  ],
  deps = [
    ':timeline_grammar_cc'
  ],
)

proto_library(
  name = "timeline_grammar",
  srcs = [
    "timeline_grammar.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "timeline_grammar_cc",
  deps = [
    ":timeline_grammar",
  ],
)
