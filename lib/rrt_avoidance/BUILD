package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'rrt_avoidance',
  srcs = glob([
    'rrt_avoidance.cc',
  ]),
  hdrs = glob([
    'rrt_avoidance.h',
  ]),
  deps = [
    ':rrt_avoidance_tools',
    '//lib/mission_manager:mission_commands_cpp',

    '@eigen',
    '@gtest//:gtest',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
  ],
)

cc_test(
  name = 'rrt_avoidance_test',
  srcs = [
    'testing/rrt_avoidance_test.cc',
  ],
  deps = [
    ':rrt_avoidance',
     '//lib/physics_structs:physics_structs',
    '@gtest//:gtest',
    '@matplotlibcpp//:matplotlibcpp',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-unused-function',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
    '-Wno-sign-compare',
  ],
)

cc_library(
  name = 'rrt_avoidance_tools',
  srcs = [
    'birrt/birrt.cc',
    '2dplane/2dplane.cpp',
    '2dplane/GridStateSpace.cpp',
    '2dplane/ObstacleGrid.cpp',
  ],
  hdrs = [
    'birrt/birrt.h',
    '2dplane/2dplane.hpp',
    '2dplane/GridStateSpace.hpp',
    '2dplane/ObstacleGrid.hpp',
    '2dplane/PlaneStateSpace.hpp',
    'planning/Path.hpp',
    'StateSpace.hpp',
    'Tree.hpp',
    'utils/util.hpp',
  ],
  deps = [
    '//lib/physics_structs:physics_structs',

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
    '-Wno-unused-function',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
  ],
)

cc_test(
  name = 'bidirectional_test',
  srcs = [
    'testing/birrt_test.cc',
  ],
  deps = [
    ':rrt_avoidance_tools',
    '@gtest//:gtest',
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
  name = 'tree_test',
  srcs = [
    'testing/tree_test.cc',
  ],
  deps = [
    ':rrt_avoidance_tools',
    '@gtest//:gtest_main',
  ],
  copts = [
    '-Wno-unused-variable',
    '-Wno-reorder',
    '-Wno-overloaded-virtual',
    '-Wno-sign-compare',
    '-Wno-unused-function',
  ],
)

