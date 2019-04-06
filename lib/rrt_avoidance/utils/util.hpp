#pragma once

#include <Eigen/Dense>

namespace lib {
namespace rrt_avoidance {

template <typename T>
bool inRange(T n, T min, T max) {
    return (n >= min) && (n <= max);
}

}  // namespace rrt_avoidance
}  // namespace lib
