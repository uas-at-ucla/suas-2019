package(default_visibility = ['//tools/cpp:__pkg__'])

filegroup(
  name = 'gcc',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:gcc',
    'arm-linux-gnueabihf-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:ar',
    'arm-linux-gnueabihf-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:ld',
    'arm-linux-gnueabihf-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:nm',
    'arm-linux-gnueabihf-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:objcopy',
    'arm-linux-gnueabihf-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:objdump',
    'arm-linux-gnueabihf-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:strip',
    'arm-linux-gnueabihf-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    '@gcc_linaro_arm_linux_gnueabihf_raspbian_repo//:as',
    'arm-linux-gnueabihf-as',
  ],
)

filegroup(
  name = 'tool-wrappers',
  srcs = [
    ':ar',
    ':as',
    ':gcc',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
  ],
)
