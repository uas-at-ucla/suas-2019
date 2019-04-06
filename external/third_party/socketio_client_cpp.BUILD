cc_library(
  name = 'socketio_client_cpp',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/**/*.cpp',
  ]),
  hdrs = glob([
    'src/**/*.h',
  ]),
  includes = [
    'src'
  ],
  deps = [
    '@websocketpp//:websocketpp',
    '@rapidjson//:rapidjson',
    '@boost//:smart_ptr',
    '@boost//:lexical_cast',
    '@boost//:asio',
  ],
)
