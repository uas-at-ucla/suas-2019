cc_library(
  name = 'ground2drone_visitor',
  visibility = ['//visibility:public'],
  srcs = [
    'ground2drone_visitor.cc'
  ],
  hdrs = [
    'ground2drone_visitor.h'
  ],
  deps = [
    '//src/controls/ground_server/timeline:timeline_grammar_cc'
  ],
)

cc_binary(
  name = 'ground2drone_visitor_test',
  srcs = [
    'ground2drone_visitor_test.cc',
  ],
  deps = [
    ':ground2drone_visitor',
    '@gtest//:googletest_main',
  ],
)
