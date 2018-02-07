workspace(name = "com_uclauas")

new_git_repository(
  name = 'gcc_linaro_arm_linux_gnueabihf_raspbian_repo',
  remote = 'https://github.com/uas-at-ucla/raspi-toolchain.git',
  build_file = 'compilers/gcc_linaro_arm_linux_gnueabihf_raspbian.BUILD',
  commit = 'f1c0b5666ab9fdb0cf491a5e18cea312a077f34b',
)

new_git_repository(
  name = 'websocketpp',
  remote = 'https://github.com/zaphoyd/websocketpp.git',
  build_file = 'third_party/websocketpp.BUILD',
  commit = '378437aecdcb1dfe62096ffd5d944bf1f640ccc3',
)

new_git_repository(
  name = 'rapidjson',
  remote = 'https://github.com/Tencent/rapidjson.git',
  build_file = 'third_party/rapidjson.BUILD',
  commit = 'daabb88e001f562e1f7df5f44d7fed32a0c107c2',
)

new_git_repository(
  name = 'rpi_ws281x',
  remote = 'https://github.com/uas-at-ucla/rpi_ws281x.git',
  build_file = 'third_party/rpi_ws281x.BUILD',
  commit = 'e3345674e5405777b61f4f509cd817d4923e99e4',
)

new_git_repository(
  name = 'socketio_client_cpp',
  remote = 'https://github.com/socketio/socket.io-client-cpp.git',
  build_file = 'third_party/socketio_client_cpp.BUILD',
  commit = '6063cb1d612f6ca0232d4134a018053fb8faea20',
)

new_http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.8.0.zip",
    sha256 = "f3ed3b58511efd272eb074a3a6d6fb79d7c2e6a0e374323d1e6bcbcc1ef141bf",
    build_file = "third_party/gtest.BUILD",
    strip_prefix = "googletest-release-1.8.0",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "239ce40e42ab0e3fe7ce84c2e9303ff8a277c41a",
    remote = "https://github.com/nelhage/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
