cc_library(
  name = 'libgphoto2',
  visibility = ['//visibility:public'],
  srcs = glob([
    'libgphoto2/*.c',
  ]),
  hdrs = glob([
    'libgphoto2/*.h',
    'gphoto2/*.h',
    'config.h',
  ]),
  includes = [
    'gphoto2',
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
