#include "battery.h"

Battery::Battery(const double capacity, const double drone_weight,
                 const double ugv_weight) :
    capacity_(capacity),
    drone_weight_(drone_weight),
    drop_off_(kLiPo),
    ugv_weight_(ugv_weight) {
  remaining_capacity_ = capacity;
}
Battery::Battery(const double capacity, const double drone_weight,
                 double drop_off, const double ugv_weight) :
    capacity_(capacity),
    drone_weight_(drone_weight),
    drop_off_(drop_off),
    ugv_weight_(ugv_weight) {
  remaining_capacity_ = capacity;
}
Battery::Battery() :
    capacity_(0),
    drone_weight_(0),
    drop_off_(0),
    ugv_weight_(0) {
  remaining_capacity_ = 0;
}
double Battery::GetRemainingCapacity() const { return remaining_capacity_; }
void Battery::ChangeRemainingCapacity(double delta_change) {
  remaining_capacity_ -= delta_change;
}
double Battery::MotorCurrentDraw(Eigen::Vector3d acceleration) {
  // change magnitude to acceleration vector
  // the air resistance the drone experiences is not factored in to this
  // calculation in later iterations it would be ideal to include it f =
  // mass*acceleration so add the acceleration; need to verify that the units of
  // acceleration are correct
  const double kPoundsToGrams = 453.592;
  const double kGravity = 9.8;

  acceleration(2) += kGravity; // factor in the force of gravity
  double thrustRequired;
  if (ugv_present_)
    thrustRequired =
        (drone_weight_ + ugv_weight_) * kPoundsToGrams * acceleration.norm();
  else
    thrustRequired = drone_weight_ * kPoundsToGrams * acceleration.norm();
  // thrustRequired must be in grams
  // original calc // double thrustRequired =
  // drone_weight_*(poundsToGrams+acceleration_magnitude);  //converts lbs to
  // grams adds the thrust required for a certain acceleration
  double current = .023 + .00129 * thrustRequired +
                   6.75 * pow(10, -8) *
                       pow(thrustRequired,
                           2); // the best fit equation from current vs thrust
  return current;
}
bool Battery::CanSourceCurrent() const {
  return remaining_capacity_ / capacity_ >= drop_off_;
}
void Battery::SetUGV(bool present) { ugv_present_ = present; }
bool Battery::GetUGVState() const { return ugv_present_; }