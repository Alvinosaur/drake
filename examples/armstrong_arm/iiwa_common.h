#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
