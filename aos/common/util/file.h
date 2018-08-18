#pragma once

#include <string>

namespace aos {
namespace util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
::std::string ReadFileToStringOrDie(const ::std::string &filename);

}  // namespace util
}  // namespace aos

