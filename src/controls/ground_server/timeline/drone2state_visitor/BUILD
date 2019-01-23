cc_library(
  name = 'drone2state_visitor',
  visibility = ['//visibility:public'],
  srcs = [
    'drone2state_visitor.cc'
  ],
  hdrs = [
    'drone2state_visitor.hh'
  ],
  deps = [
    '//src/controls/ground_server/timeline/drone_state_machine:drone_state_machine',
    '//src/controls/ground_server/timeline:timeline_grammar_cc'
  ]
)

cc_binary(
  name = 'drone2state_visitor_test',
  visibility = ['//visibility:public'],
  srcs = [ 'drone2state_visitor_test.cc' ],
  deps = [ ':drone2state_visitor' ]
)
