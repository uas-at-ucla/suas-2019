#pragma once

#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

// Uses disjoint sets to group ranges into disjoint RangeImage.
// ranges that overlap are grouped into the same output RangeImage.
BlobList FindBlobs(const RangeImage &rimg);

}  // namespace vision
}  // namespace aos

