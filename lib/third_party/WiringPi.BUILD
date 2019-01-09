cc_library(
  name = 'WiringPi',
  visibility = ['//visibility:public'],
  srcs = glob([
    'wiringPi/*.c',
    'wiringPiD/*.c',
  ]),
  hdrs = glob([
    'wiringPi/*.h',
    'wiringPiD/*.h',
    'version.h',
  ]),
  includes = [
    'wiringPi',
  ],
  copts = [
    '-Wno-unused-function',
    '-Wno-int-to-pointer-cast',
    '-Wno-format-nonliteral',
  ],
)
