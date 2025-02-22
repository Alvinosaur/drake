#include "drake/examples/armstrong_arm/iiwa_common.h"

#include <cmath>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  const int num_joints = 18;
  Kp->resize(num_joints);
  *Kp = 100 * Eigen::VectorXd::Ones(num_joints);
  // *Kp << 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(num_joints);
}


}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
