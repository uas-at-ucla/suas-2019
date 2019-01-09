cc_library(
  name = 'libzmq',
  visibility = ["//visibility:public"],
  srcs = glob([
    'src/**/*.cpp',
    'src/**/*.c',
  ]),
  hdrs = glob([
    'src/**/*.hpp',
    'src/**/*.h',
    'include/**/*.h',
  ]),
  includes = [
    'include',
    'src',
  ],
  copts = [
    '-Wno-macro-redefined'
  ],
)
