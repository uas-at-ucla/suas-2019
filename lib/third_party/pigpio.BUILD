licenses(['notice'])

cc_library(
  name = 'pigpio',
  visibility = ['//visibility:public'],
  srcs = glob([
    'pigpio.c',
    'pigpiod.c',
    'pigpiod_if2.c',
    'command.c',
  ]),
  hdrs = glob([
    'pigpio.h',
    'pigpiod.h',
    'command.h',
    'pigpiod_if2.h',
    'custom.cext',
  ]),
  includes = ['.'],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-unused-variable',
    '-Wno-unused-function',
    '-Wno-format-nonliteral',
    '-Wno-format',
    '-Wno-tautological-compare',
    '-Wno-int-to-pointer-cast',
    '-Wno-incompatible-pointer-types-discards-qualifiers',
    '-Wno-implicit-function-declaration',
    '-Wno-pointer-sign',
    '-Wno-sign-compare',
    '-Wno-return-type',
    '-Wno-constant-conversion',
    '-Wno-type-limits',
    '-Wno-error',
  ],
)
