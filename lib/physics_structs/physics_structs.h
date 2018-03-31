#ifndef SPINNY_LIB_PHYSICS_STRUCTS_PHYSICS_STRUCTS
#define SPINNY_LIB_PHYSICS_STRUCTS_PHYSICS_STRUCTS

#include <cmath>


namespace physical_constants {
constexpr double kRadiusOfEarthInMeters = 6.3781e6;
}  // namespace physical_constants

struct Vector3D {
  double x;
  double y;
  double z;
};

struct Position3D {
  double latitude;
  double longitude;
  double altitude;
};

struct Obstacle {
  Position3D position;
  double radius;
};

Vector3D& operator+=(Vector3D& vector, double scalar);
Vector3D& operator-=(Vector3D& vector, double scalar);
Vector3D& operator*=(Vector3D& vector, double scalar);
Vector3D& operator/=(Vector3D& vector, double scalar);

double DegreesToRadians(double degrees);

double GetDistance2D(Position3D start, Position3D end);
double GetDistance3D(Position3D start, Position3D end);
Vector3D PointTowards(Position3D start, Position3D end);
double GetMagnitude(Vector3D);


#endif  // SPINNY_LIB_PHYSICS_STRUCTS_PHYSICS_STRUCTS
