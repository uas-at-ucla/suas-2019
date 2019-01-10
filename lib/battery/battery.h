#include "math.h"
#include <Eigen/Dense>
class Battery {
 public:
  // capacity in ampHours //droneWeight is in pounds //dropoff the percentage of
  // the amount of capacity that causes the battery to not supply enough voltage
  Battery(const double capacity, const double drone_weight,
          const double ugv_weight);
  Battery(const double capacity, const double drone_weight, double drop_off,
          double ugv_weight);
  Battery();
  double GetRemainingCapacity() const;
  void ChangeRemainingCapacity(
      double delta_change); // change remaining capacity by a given amount
  double MotorCurrentDraw(
      ::Eigen::Vector3d acceleration); // need to check why this value cannot be
                                       // negative in the motion profile class
  void SetUGV(bool attached);
  bool GetUGVState() const;
  bool CanSourceCurrent() const;
  const double kLiPo = .125;

 private:
  const double capacity_;
  const double drone_weight_;
  const double drop_off_;
  const double ugv_weight_; // most constructors default this to
  bool ugv_present_ = true;
  double remaining_capacity_;
};