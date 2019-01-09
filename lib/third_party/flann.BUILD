cc_library(
  name = 'flann',
  visibility = ["//visibility:public"],
  srcs = glob([
    'src/cpp/flann/**/*.cpp',
  ], exclude = [
    'src/cpp/flann/mpi/**/*.cpp',
  ]),
  hdrs = glob([
    'src/cpp/flann/**/*.h',
    'src/cpp/flann/**/*.hpp',
  ], exclude = [
    'src/cpp/flann/mpi/**/*.h',
    'src/cpp/flann/mpi/**/*.hpp',
  ]),
  includes = [
    'src/cpp',
  ],
  copts = [
    '-Wno-missing-field-initializers',
  ],
)
