#include "base64_tools.h"

namespace lib {
namespace base64_tools {

// Taken from here:
// https://renenyffenegger.ch/notes/development/Base64/Encoding-and-decoding-base-64-with-cpp
bool IsBase64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

::std::string Decode(::std::string const &encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;

  while (in_len-- && (encoded_string[in_] != '=') &&
         IsBase64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_];
    in_++;
    if (i == 4) {
      for (i = 0; i < 4; i++)
        char_array_4[i] = kBase64Chars.find(char_array_4[i]);

      char_array_3[0] =
          (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] =
          ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++)
        ret += char_array_3[i];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 4; j++)
      char_array_4[j] = 0;

    for (j = 0; j < 4; j++)
      char_array_4[j] = kBase64Chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] =
        ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++)
      ret += char_array_3[j];
  }

  return ret;
}

// Taken from here:
// https://stackoverflow.com/questions/342409/how-do-i-base64-encode-decode-in-c
::std::string Encode(::std::string const &input) {
  const unsigned char *src =
      reinterpret_cast<unsigned const char *>(input.c_str());
  size_t len = input.length();

  unsigned char *out, *pos;
  const unsigned char *end, *in;

  size_t olen;

  olen = 4 * ((len + 2) / 3); /* 3-byte blocks to 4-byte */

  if (olen < len)
    return std::string(); /* integer overflow */

  std::string outStr;
  outStr.resize(olen);
  out = (unsigned char *)&outStr[0];

  end = src + len;
  in = src;
  pos = out;
  while (end - in >= 3) {
    *pos++ = kBase64Chars[in[0] >> 2];
    *pos++ = kBase64Chars[((in[0] & 0x03) << 4) | (in[1] >> 4)];
    *pos++ = kBase64Chars[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
    *pos++ = kBase64Chars[in[2] & 0x3f];
    in += 3;
  }

  if (end - in) {
    *pos++ = kBase64Chars[in[0] >> 2];
    if (end - in == 1) {
      *pos++ = kBase64Chars[(in[0] & 0x03) << 4];
      *pos++ = '=';
    } else {
      *pos++ = kBase64Chars[((in[0] & 0x03) << 4) | (in[1] >> 4)];
      *pos++ = kBase64Chars[(in[1] & 0x0f) << 2];
    }
    *pos++ = '=';
  }

  return outStr;
}

} // namespace base64_tools
} // namespace lib
