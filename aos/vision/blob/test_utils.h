#pragma once

#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

// For tests. Loads a RangeImage from a constant string.
//
// For example this should look something like this:
//  first and final returns will be stripped.
//
// R"(
//  ---  ---
//   -----
//     --
// )"
RangeImage LoadFromTestData(int mini, const char *data);

}  // namespace vision
}  // namespace aos

