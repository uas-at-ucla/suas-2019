#include "lib/physics_structs/physics_structs.h"

#include <iostream>

Vector3D &operator+=(Vector3D &vector, double scalar) {
  vector.x += scalar;
  vector.y += scalar;
  vector.z += scalar;

  return vector;
}

Vector3D &operator-=(Vector3D &vector, double scalar) {
  vector.x -= scalar;
  vector.y -= scalar;
  vector.z -= scalar;

  return vector;
}

Vector3D &operator*=(Vector3D &vector, double scalar) {
  vector.x *= scalar;
  vector.y *= scalar;
  vector.z *= scalar;

  return vector;
}

Vector3D &operator/=(Vector3D &vector, double scalar) {
  vector.x /= scalar;
  vector.y /= scalar;
  vector.z /= scalar;

  return vector;
}

double DegreesToRadians(double degrees) { return degrees / 180.0 * M_PI; }

double GetMagnitude(Vector3D vector) {
  return sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
}

double GetDistance2D(lib::Position3D start, lib::Position3D end) {
  // Uses haversine formula.

  start.latitude = DegreesToRadians(start.latitude);
  start.longitude = DegreesToRadians(start.longitude);
  end.latitude = DegreesToRadians(end.latitude);
  end.longitude = DegreesToRadians(end.longitude);

  double d_latitude = end.latitude - start.latitude;
  double d_longitude = end.longitude - start.longitude;

  double a = pow(sin(d_latitude / 2), 2) + cos(start.latitude) *
                                               cos(end.latitude) *
                                               pow(sin(d_longitude / 2), 2);

  double c = 2 * asin(sqrt(a));
  double r = physical_constants::kRadiusOfEarthInMeters;
  return c * r;
}

double GetDistance3D(lib::Position3D start, lib::Position3D end) {
  double ground_distance = GetDistance2D(start, end);
  double climb_distance = end.altitude - start.altitude;

  return sqrt(pow(ground_distance, 2) + pow(climb_distance, 2));
}

Vector3D PointTowards(lib::Position3D start, lib::Position3D end) {
  double dx = GetDistance2D({start.latitude, 0, 0}, {end.latitude, 0, 0});
  double dy = GetDistance2D({0, start.longitude, 0}, {0, end.longitude, 0});
  double dz = start.altitude - end.altitude; // In NED coordinates.

  dx *= end.latitude > start.latitude ? 1 : -1;
  dy *= end.longitude > start.longitude ? 1 : -1;

  Vector3D pointing_vector = {dx, dy, dz};

  // Make sure we don't divide by zero.
  double magnitude = ::std::abs(GetMagnitude(pointing_vector));
  pointing_vector.x = magnitude == 0 ? 0 : pointing_vector.x / magnitude;
  pointing_vector.y = magnitude == 0 ? 0 : pointing_vector.y / magnitude;
  pointing_vector.z = magnitude == 0 ? 0 : pointing_vector.z / magnitude;

  return pointing_vector;
}
