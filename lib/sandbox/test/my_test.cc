#include "ros/ros.h"
#include "gtest/gtest.h"

TEST(FactorialTest, Negative) {
  EXPECT_EQ(1, 1);

  ::ros::Time::init();
  ::ros::Rate loop(100);
  loop.sleep();
}
