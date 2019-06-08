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
    '//src/controls/ground_controls/timeline:timeline_grammar_cc',
    '//lib/rrt_avoidance:rrt_avoidance',
    '//lib/mission_manager:mission_commands_cpp',
  ],
)

cc_binary(
  name = 'ground2drone_visitor_test',
  srcs = [
    'ground2drone_visitor_test.cc',
  ],
  deps = [
    ':ground2drone_visitor',
    '@gtest//:gtest_main',
  ],
)
