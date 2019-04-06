cc_library(
  name = 'gphoto2pp',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/**/*.cpp',
  ]),
  hdrs = glob([
    'include/**/*.hpp',
  ]),
  includes = [
    'include',
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-implicit-function-declaration',
    '-Wno-incompatible-pointer-types-discards-qualifiers',
    '-Wno-format-nonliteral',
    '-Wno-int-conversion',
    '-Wno-sign-compare',
    '-Wno-#warnings',
  ],
)
