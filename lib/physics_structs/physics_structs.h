#pragma once

#include <cmath>

namespace lib {

struct Position3D {
  double latitude;
  double longitude;
  double altitude;
};
} // namespace lib
namespace physical_constants {
constexpr double kRadiusOfEarthInMeters = 6.3781e6;
} // namespace physical_constants

struct Vector3D {
  double x;
  double y;
  double z;
};

struct Obstacle {
  lib::Position3D position;
  double radius;
};

Vector3D &operator+=(Vector3D &vector, double scalar);
Vector3D &operator-=(Vector3D &vector, double scalar);
Vector3D &operator*=(Vector3D &vector, double scalar);
Vector3D &operator/=(Vector3D &vector, double scalar);

double DegreesToRadians(double degrees);

double GetDistance2D(lib::Position3D start, lib::Position3D end);
double GetDistance3D(lib::Position3D start, lib::Position3D end);
Vector3D PointTowards(lib::Position3D start, lib::Position3D end);
double GetMagnitude(Vector3D);
