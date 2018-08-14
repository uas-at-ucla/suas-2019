#pragma once

#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

RangeImage Transpose(const RangeImage &img);
inline std::vector<RangeImage> Transpose(const std::vector<RangeImage> &imgs) {
  std::vector<RangeImage> out;
  out.reserve(imgs.size());
  for (const auto &img : imgs) out.push_back(Transpose(img));
  return out;
}

}  // namespace vision
}  // namespace aos

