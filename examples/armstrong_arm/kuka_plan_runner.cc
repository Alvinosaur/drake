/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a lcmt_robot_plan message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 18;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant, multibody::ModelInstanceIndex robot_model_index)
      : plant_(plant), robot_model_index_(robot_model_index) {
    context_ = plant_.CreateDefaultContext();
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
  }

  void Run() {
    // int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    // int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      cur_time_us = iiwa_status_.utime;

      iiwa_command.utime = iiwa_status_.utime;

      for (int joint = 0; joint < kNumJoints; joint++) {
        iiwa_command.joint_position[joint] = target_joints_(joint);
      }

      lcm_.publish(kLcmCommandChannel, &iiwa_command);
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  // void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
  //                 const lcmt_robot_plan* plan) {
  //   std::cout << "New plan received." << std::endl;
  //   if (iiwa_status_.utime == -1) {
  //     std::cout << "Discarding plan, no status message received yet"
  //               << std::endl;
  //     return;
  //   } else if (plan->num_states < 2) {
  //     std::cout << "Discarding plan, Not enough knot points." << std::endl;
  //     return;
  //   }

  //   std::vector<Eigen::MatrixXd> knots(plan->num_states,
  //                                      Eigen::MatrixXd::Zero(kNumJoints, 1));
  //   std::cout << "Num plan states: " << plan->num_states << std::endl;                               
  //   for (int i = 0; i < plan->num_states; ++i) {
  //     const auto& state = plan->plan[i];
  //     for (int j = 0; j < state.num_joints; ++j) {
  //       std::cout << "Num plan states: " << plan->num_states << std::endl;     
  //       std::cout << "Joint " << j << " value: " << state.joint_position[j] << std::endl;
  //       if (!plant_.HasJointNamed(state.joint_name[j])) {
  //         std::cout << "SKIPPED BECAUSE WRONG NAME?? " << state.joint_name[j] << std::endl;
  //         continue;
  //       }
  //       const multibody::Joint<double>& joint =
  //           plant_.GetJointByName(state.joint_name[j]);
  //       DRAKE_DEMAND(joint.num_positions() == 1);
  //       const int idx = joint.position_start();
  //       DRAKE_DEMAND(idx < kNumJoints);

  //       // Treat the matrix at knots[i] as a column vector.
  //       if (i == 0) {
  //         // Always start moving from the position which we're
  //         // currently commanding.
  //         DRAKE_DEMAND(iiwa_status_.utime != -1);
  //         knots[0](idx, 0) = iiwa_status_.joint_position_commanded[j];

  //       } else {
  //         knots[i](idx, 0) = state.joint_position[j];
  //       }
  //     }
  //   }

  //   std::cout << "Knot points: " << std::endl;
  //   for (int i = 0; i < plan->num_states; ++i) {
  //     std::cout << knots[i] << std::endl;
  //   }

  //   std::vector<double> input_time;
  //   for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
  //     input_time.push_back(plan->plan[k].utime / 1e6);
  //   }
  //   const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
  //   plan_.reset(new PiecewisePolynomial<double>(
  //       PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //           input_time, knots, knot_dot, knot_dot)));
  //   ++plan_number_;
  // }

void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan* plan) {
    std::cout << "New plan received." << std::endl;
    const auto& state = plan->plan[0];
    for (int j = 0; j < state.num_joints; ++j) {
      std::cout << state.joint_position[j] << std::endl;
      target_joints_[j] = state.joint_position[j];
    }
  }

  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan*) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<systems::Context<double>> context_;
  // int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  Eigen::VectorXd target_joints_ = Eigen::VectorXd::Zero(kNumJoints);
  multibody::ModelInstanceIndex robot_model_index_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  auto robot_model_index = multibody::Parser(&plant).AddModelFromFile("/home/armstrong/catkin_ws/src/vision/robot_meshes/shotwell_real_sidecar_robot_decomposed.sdf");
  plant.Finalize();

  RobotPlanRunner runner(plant, robot_model_index);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::kuka_iiwa_arm::do_main();
}
