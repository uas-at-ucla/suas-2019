#pragma once

#include <chrono>
#include <string>
#include <thread>

namespace lib {
namespace base64_tools {
namespace {
const ::std::string kBase64Chars = //
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"   //
    "abcdefghijklmnopqrstuvwxyz"   //
    "0123456789+/";
} // namespace

::std::string Decode(::std::string const &encoded_string);
::std::string Encode(const unsigned char *src, size_t len);
::std::string Encode(::std::string str);
bool IsBase64(unsigned char c);

} // namespace phased_loop
} // namespace lib
