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
::std::string Encode(::std::string const &str);
bool IsBase64(unsigned char c);

} // namespace base64_tools
} // namespace lib
