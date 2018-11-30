cc_library(
  name = 'executor',
  visibility = ['//visibility:public'],
  srcs = [
    'executor.cc'
  ],
  hdrs = [
    'executor.h'
  ],
  deps = [
    '//src/controls/ground_server/timeline:timeline_grammar_cc'
  ],
)

cc_binary(
  name = 'executor_test',
  srcs = [
    'executor_test.cc',
  ],
  deps = [
    ':executor',
    '@gtest//:googletest_main',
  ],
)
