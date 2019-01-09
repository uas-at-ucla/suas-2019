cc_library(
  name = 'sodium',
  visibility = ['//visibility:public'],
  srcs = glob([
    'sodium/**/*.cpp',
  ]),
  hdrs = glob([
    'sodium/**/*.h',
  ]),
  includes = [
    '.',
  ],
  deps = [
    '@boost//:optional',
    '@boost//:intrusive_ptr',
  ],
  copts = [
    '-Wno-unused-parameter',
  ],
)
