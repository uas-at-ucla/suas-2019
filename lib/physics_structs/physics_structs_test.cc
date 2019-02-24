#include "gtest/gtest.h"

#include "lib/physics_structs/physics_structs.h"

namespace testing {

TEST(PhysicsStructsTest, VectorOperations) {
  Vector3D vector = {5, 10, 15};
  EXPECT_EQ(vector.x, 5);
  EXPECT_EQ(vector.y, 10);
  EXPECT_EQ(vector.z, 15);

  vector += 5;

  EXPECT_EQ(vector.x, 10);
  EXPECT_EQ(vector.y, 15);
  EXPECT_EQ(vector.z, 20);

  vector -= 5;

  EXPECT_EQ(vector.x, 5);
  EXPECT_EQ(vector.y, 10);
  EXPECT_EQ(vector.z, 15);

  vector *= 2;

  EXPECT_EQ(vector.x, 10);
  EXPECT_EQ(vector.y, 20);
  EXPECT_EQ(vector.z, 30);

  vector /= 2;

  EXPECT_EQ(vector.x, 5);
  EXPECT_EQ(vector.y, 10);
  EXPECT_EQ(vector.z, 15);
}

TEST(PhysicsStructsTest, Distance2DLongDistanceTest) {
  double kDistanceTolerance = 5e3;

  lib::Position3D kLosAngelesCoordinate = {40.730610, -73.93524233, 0};
  lib::Position3D kNewYorkCoordinate = {34.052235, -118.243683, 0};
  double kDistanceBetweenLosAngelesAndNewYorkInMeters = 3.95026e6;

  double calculated_distance =
      GetDistance2D(kLosAngelesCoordinate, kNewYorkCoordinate);
  EXPECT_GE(calculated_distance,
            kDistanceBetweenLosAngelesAndNewYorkInMeters - kDistanceTolerance);

  EXPECT_LE(calculated_distance,
            kDistanceBetweenLosAngelesAndNewYorkInMeters + kDistanceTolerance);
}

TEST(PhysicsStructsTest, PointTowardsTest) {
  double kAngleTolerance = 1e-6;

  lib::Position3D start = {0, 0, 0};
  lib::Position3D end = {1, 1, 0};

  Vector3D pointing_to = PointTowards(start, end);

  EXPECT_LE(pointing_to.x, 0.707107 + kAngleTolerance);
  EXPECT_GE(pointing_to.x, 0.707107 - kAngleTolerance);
}

} // namespace testing
