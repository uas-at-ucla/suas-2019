package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'rrt_avoidance',
  srcs = glob([
    '2dplane/**/*.cpp',
  ]),
  hdrs = glob([
    '**/*.hpp',
  ]),
  deps = [
    '@eigen',
    '@boost//:functional',
    '@boost//:config',
    '@boost//:core',
    '@boost//:utility',
    '@boost//:assert',
    '@boost//:type_traits',
    '@boost//:algorithm',
    '@flann//:flann',
    '@lz4//:lz4',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
  ],
)

cc_test(
  name = 'rrt_avoidance_bidirectional_test',
  srcs = [
    'BiRRTTest.cpp',
  ],
  deps = [
    ':rrt_avoidance',
    '@gtest//:googletest',
    '@matplotlibcpp//:matplotlibcpp',
     '//lib/physics_structs:physics_structs',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
    '-Wno-sign-compare',
  ],
)

cc_test(
  name = 'rrt_avoidance_tree_test',
  srcs = [
    'TreeTest.cpp',
  ],
  deps = [
    ':rrt_avoidance',
    '@gtest//:googletest_main',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
    '-Wno-sign-compare',
    '-Wno-unused-function',
  ],
)

