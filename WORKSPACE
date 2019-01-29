workspace(name = "com_uclauas")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

new_git_repository(
  name = 'raspi-toolchain',
  remote = 'https://github.com/uas-at-ucla/raspi-toolchain.git',
  build_file = 'compilers/raspi-toolchain.BUILD',
  commit = 'a3147acb2a75dcc54d33665882c2e399229d4824',
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
  remote = 'https://github.com/uas-at-ucla/socket.io-client-cpp.git',
  build_file = 'third_party/socketio_client_cpp.BUILD',
  commit = '28d2105f8052641498d305cb5ea02a73e60eb91a',
)

new_git_repository(
  name = 'mavlink_v1',
  remote = 'https://github.com/mavlink/c_library_v1.git',
  build_file = 'third_party/mavlink_v1.BUILD',
  commit = 'c0e0ebf80ce0e0d497245f94869eb9c5e817114f',
)

new_git_repository(
  name = 'mavlink_v2',
  remote = 'https://github.com/mavlink/c_library_v2.git',
  build_file = 'third_party/mavlink_v2.BUILD',
  commit = '748192f661d0df3763501cfc432861d981952921',
)

new_git_repository(
  name = 'libmavconn',
  remote = 'https://github.com/mavlink/mavros.git',
  build_file = 'third_party/libmavconn.BUILD',
  commit = '306f7ee6e0d90fa2c63f4a821b8f09b0f02219b9',
)

new_git_repository(
  name = 'eigen',
  remote = 'https://github.com/PX4/eigen.git',
  build_file = 'third_party/eigen.BUILD',
  commit = 'e7850ed81f9c469e02df496ef09ae32ec0379b71',
)

new_git_repository(
  name = 'console_bridge',
  remote = 'https://github.com/uas-at-ucla/console_bridge.git',
  build_file = 'third_party/console_bridge.BUILD',
  commit = '57f9d41e3402d317fec85b3cc531d9dc4437bf45',
)

new_git_repository(
  name = "libzmq",
  commit = "b1fd3b2740f4b18ee25aefe193b31e319fcfbbe5",
  build_file = "third_party/libzmq.BUILD",
  remote = "https://github.com/uas-at-ucla/libzmq.git",
)

new_git_repository(
  name = "cppzmq",
  commit = "a96e0ded6456757ed74947cec2b5d2b875f9dab4",
  build_file = "third_party/cppzmq.BUILD",
  remote = "https://github.com/zeromq/cppzmq.git",
)

new_git_repository(
  name = "sodium",
  commit = "e25e2884e2543f608de34fb92d719f0724d5a5c7",
  build_file = "third_party/sodium.BUILD",
  remote = "https://github.com/SodiumFRP/sodium-cxx.git",
)

new_git_repository(
  name = "flann",
  commit = "27f9d9333f970b4bccaa95fa06f5a8344e931dc1",
  build_file = "third_party/flann.BUILD",
  remote = "https://github.com/mariusmuja/flann.git",
)

new_git_repository(
  name = "lz4",
  commit = "b5233d3726b416b1176c71483d20b4c543851c6f",
  build_file = "third_party/lz4.BUILD",
  remote = "https://github.com/lz4/lz4.git",
)

new_git_repository(
  name = "matplotlibcpp",
  commit = "9523030a3dc11c05b6cc38743299e4453ec36458",
  build_file = "third_party/matplotlibcpp.BUILD",
  remote = "https://github.com/uas-at-ucla/matplotlib-cpp.git",
)

git_repository(
  name = "gtest",
  commit = "fe14e3030737509c2b9f9adad80ff56f80e988a2",
  remote = "https://github.com/abseil/googletest.git"
)

git_repository(
  name = "com_github_nelhage_rules_boost",
  commit = "6d6fd834281cb8f8e758dd9ad76df86304bf1869",
  remote = "https://github.com/nelhage/rules_boost",
)


new_git_repository(
  name = "WiringPi",
  commit = "0f7d03d9f04c226192c299dafe03e1d56ec219c9",
  remote = "https://github.com/WiringPi/WiringPi.git",
  build_file = "third_party/WiringPi.BUILD",
)

new_git_repository(
  name = "spdlog",
  commit = "93d41b2c0ecd0db7075e2386596ce39cb20546c9",
  remote = "https://github.com/gabime/spdlog.git",
  build_file = "third_party/spdlog.BUILD",
)

new_git_repository(
  name = "pigpio",
  commit = "934874be2fa34a525beb33e8cb75e378df587860",
  remote = "https://github.com/joan2937/pigpio.git",
  build_file = "third_party/pigpio.BUILD",
)

new_git_repository(
  name = "libgphoto2",
  commit = "a9f979459f0a64ff356318cd48b6f5b817850ec5",
  remote = "https://github.com/uas-at-ucla/libgphoto2.git",
  build_file = "third_party/libgphoto2.BUILD",
)

git_repository(
  name = "PX4_sitl",
  remote = "https://github.com/uas-at-ucla/PX4_sitl.git",
  commit = "e9aa8c07cd9b62e484512826ea4bdab396be3801",
)

new_git_repository(
  name = "gphoto2pp",
  commit = "da6db6f79edc4291f7a5ff38d825b8f3102d6270",
  remote = "https://github.com/maldworth/gphoto2pp.git",
  build_file = "third_party/gphoto2pp.BUILD"
)

new_git_repository(
  name = "ros_bazel",
  commit = "652439a8061403f434701b61913547febca46a38",
  remote = "https://github.com/uas-at-ucla/ros_bazel.git",
  build_file = "third_party/ros_bazel.BUILD"
)

new_local_repository(
    name = "python_linux",
    path = "/usr",
    build_file_content = """
cc_library(
    name = "python27-lib",
    srcs = ["lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so"],
    hdrs = glob(["include/python2.7/*.h"]),
    includes = ["include/python2.7"],
    visibility = ["//visibility:public"]
)
    """
)

new_git_repository(
  name = "com_google_protobuf",
  remote = "https://github.com/protocolbuffers/protobuf.git",
  commit = "66dc42d891a4fc8e9190c524fd67961688a37bbe",
  build_file = 'third_party/com_google_protobuf.BUILD'
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

local_repository(
  name = "ignore_server_rxjs",
  path = "./src/ground/server/node_modules/rxjs/src",
)

local_repository(
  name = "ignore_ui_rxjs",
  path = "./src/ground/ui/node_modules/rxjs/src",
)

new_http_archive(
    name='genmsg_repo',
    build_file='external/third_party/genmsg.BUILD',
    sha256='d7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2',
    urls = ['https://github.com/ros/genmsg/archive/0.5.8.tar.gz'],
    strip_prefix='genmsg-0.5.8',
)

new_http_archive(
    name='genpy_repo',
    build_file='external/third_party/genpy.BUILD',
    sha256='35e5cd2032f52a1f77190df5c31c02134dc460bfeda3f28b5a860a95309342b9',
    urls = ['https://github.com/ros/genpy/archive/0.6.5.tar.gz'],
    strip_prefix='genpy-0.6.5',
)

new_http_archive(
    name='gencpp',
    build_file='external/third_party/gencpp.BUILD',
    sha256='d7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2',
    urls = ['https://github.com/ros/gencpp/archive/0.5.5.tar.gz'],
    strip_prefix='gencpp-0.5.5',
)
