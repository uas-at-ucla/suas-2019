#pragma once

#include <string>

namespace aos {
namespace libc {

// Thread-safe version of dirname(3).
::std::string Dirname(const ::std::string &path);

}  // namespace libc
}  // namespace aos

