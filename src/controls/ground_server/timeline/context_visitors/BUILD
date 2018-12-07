cc_library(
  name = 'context_visitor',
  visibility = ['//visibility:public'],
  srcs = [
    'context_visitor.cc'
  ],
  hdrs = [
    'context_visitor.h'
  ],
  deps = [
    '//src/controls/ground_server/timeline:timeline_grammar_cc'
  ],
)

cc_binary(
  name = 'context_visitor_test',
  srcs = [
    'context_visitor_test.cc',
  ],
  deps = [
    ':context_visitor',
    '@gtest//:gtest_main',
  ],
)
