#include "drake/manipulation/util/move_ik_demo_base.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/manipulation/util/robot_plan_utils.h"
#include "drake/multibody/parsing/parser.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/solve.h"

// #include "drake/solvers/clp_solver.h"
// #include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/nlopt_solver.h"
#include <iostream>

using namespace std;

namespace drake {
namespace manipulation {
namespace util {

using planner::ConstraintRelaxingIk;


enum Solver { IPOPT = 0, GUROBI = 1, MOSEK = 2, SNOPT = 3, NLOPT = 4 };


inline static double fRand(double min, double max)
{
    double f = static_cast<double>(rand()) / RAND_MAX;
    return min + f * (max - min);
}

void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    //Find actual angle offset
    double diffangle = fmod(val - max, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    //Find actual angle offset
    double diffangle = fmod(min - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = min - diffangle + 2 * M_PI;
  }
}

void normalizeAngle(double& val, const double& target)
{
  double new_target = target + M_PI;
  if (val > new_target)
  {
    //Find actual angle offset
    double diffangle = fmod(val - new_target, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = new_target + diffangle - 2 * M_PI;
  }

  new_target = target - M_PI;
  if (val < new_target)
  {
    //Find actual angle offset
    double diffangle = fmod(new_target - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = new_target - diffangle + 2 * M_PI;
  }
}

Eigen::VectorXd normalize_seed(const Eigen::VectorXd& seed, const Eigen::VectorXd& solution, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{
  // Make sure rotational joint values are within 1 revolution of seed; then
  // ensure joint limits are met.
  assert(solution.rows() == lb.rows() && solution.rows() == ub.rows());
  Eigen::VectorXd new_solution = solution;
  for (uint i = 0; i < lb.rows(); i++)
  {
    double target = seed(i);
    double val = new_solution(i);

    normalizeAngle(val, target);
    normalizeAngle(val, lb(i), ub(i));
    new_solution(i) = val;
  }
  return new_solution;
}


MoveIkDemoBase::MoveIkDemoBase(std::string robot_description,
                               std::string base_link,
                               std::string ik_link,
                               int print_interval)
    : robot_description_(std::move(robot_description)),
      ik_link_(std::move(ik_link)),
      print_interval_(print_interval),
      plant_(0.0),
      constraint_relaxing_ik_(robot_description_, ik_link_) {
  multibody::Parser(&plant_).AddModelFromFile(robot_description_);
  (void)base_link;
  // plant_.WeldFrames(plant_.world_frame(),
  //                   plant_.GetBodyByName(base_link).body_frame());
  plant_.Finalize();
  context_ = plant_.CreateDefaultContext();
  joint_names_ = GetJointNames(plant_);
  joint_velocity_limits_ = plant_.GetVelocityUpperLimits();
}

MoveIkDemoBase::~MoveIkDemoBase() {}

void MoveIkDemoBase::set_joint_velocity_limits(
    const Eigen::Ref<const Eigen::VectorXd>& velocity_limits) {
  DRAKE_THROW_UNLESS(velocity_limits.size() ==
                     joint_velocity_limits_.size());
  joint_velocity_limits_ = velocity_limits;
}

void MoveIkDemoBase::HandleStatus(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  status_count_++;
  plant_.SetPositions(context_.get(), q);

  if (status_count_ % print_interval_ == 1) {
    const math::RigidTransform<double> current_link_pose =
        plant_.EvalBodyPoseInWorld(
            *context_, plant_.GetBodyByName(ik_link_));
    const math::RollPitchYaw<double> rpy(current_link_pose.rotation());
    drake::log()->info("{} at: {} {}",
                       ik_link_,
                       current_link_pose.translation().transpose(),
                       rpy.vector().transpose());
  }
}



bool solve_ik(const Eigen::Vector3d& target_position_in_robot, 
const Eigen::Matrix3d& target_orientation_mat_in_robot, drake::multibody::MultibodyPlant<double>* plant, 
drake::systems::Context<double>* plant_context, const Eigen::VectorXd& seed, Eigen::VectorXd& solved_joints, const Eigen::Vector3d position_buffer = Eigen::Vector3d::Zero(), float theta_bound=0.0, bool with_joint_limits=true, int max_iter=10000, 
float max_time_s=0.1, int min_attempts=4, float tol=1e-5) {

    // TODO: DON'T HARDCODE THIS
    // index offset for correct arm joints variables
    const int left_arm_offset = 0;
    // const int right_arm_offset = 9;
    const int arm_offset = left_arm_offset;

    // cout << "Target Position: " << target_position_in_robot << endl;
    // cout << "Target Rotation: " << target_orientation_mat_in_robot << endl;
    drake::multibody::InverseKinematics ik_solver(*plant, plant_context, with_joint_limits);
    const auto& gripper_frame = plant->GetFrameByName("left_link7");
    const auto& robot_frame = plant->GetFrameByName("robot");
    ik_solver.AddPositionConstraint(
            gripper_frame, Eigen::Vector3d::Zero(), robot_frame, 
            target_position_in_robot - position_buffer, target_position_in_robot + position_buffer);

    drake::math::RotationMatrix<double> identity = drake::math::RotationMatrix<double>();
    ik_solver.AddOrientationConstraint(
            gripper_frame, identity, robot_frame, 
            drake::math::RotationMatrix<double>{target_orientation_mat_in_robot}, theta_bound);

    // Add cost of staying close to the seed
    // float seed_dist_weight = 0.1;
    // ik_solver.get_mutable_prog()->AddQuadraticCost(seed_dist_weight * (ik_solver.q().block<7,1>(arm_offset, 0) - seed).squaredNorm());

    // Collision Avoidance Constraint
    // float min_distance = 0.0;  // padding distance
    // float influence_distance_offset = 1;  // TODO: not sure what this parameter does...
    // ik_solver.AddMinimumDistanceConstraint(min_distance, influence_distance_offset);

    // try setting seed and solving
    Eigen::VectorXd lower_limits = plant->GetPositionLowerLimits().block<7,1>(arm_offset, 0);
    Eigen::VectorXd upper_limits = plant->GetPositionUpperLimits().block<7,1>(arm_offset, 0);

    // allocate time per attempt
    float max_time_per_attempt_s = max_time_s / static_cast<float>(min_attempts);

    // Set Solver Options, default to Ipopt
    Solver solver = NLOPT;
    auto solver_id = drake::solvers::IpoptSolver::id();
    // unordered_map<drake::solvers::SolverId, string> solver_to_param
    switch(solver) {
        case SNOPT: {
            solver_id = drake::solvers::SnoptSolver::id();
            // ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "max_iter", max_iter);
            // ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "max_cpu_time", max_time_per_attempt_s);
            // ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "tol", tol);
            break;
        }
        case GUROBI: {
            solver_id = drake::solvers::GurobiSolver::id();
            break;
        }
        case MOSEK: {
            solver_id = drake::solvers::MosekSolver::id();
            break;
        }
        case NLOPT: {
            solver_id = drake::solvers::NloptSolver::id();
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "max_iter", max_iter);
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "HELLOL", max_time_per_attempt_s);
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "tol", tol);
            break;

        }
        default: {
            solver_id = drake::solvers::IpoptSolver::id();
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "max_iter", max_iter);
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "max_cpu_time", max_time_per_attempt_s);
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id, "tol", tol);
            break;
        }
    }
    std::optional<drake::solvers::SolverId> solver_id_optional = solver_id;

    // TODO: Can this be parallelized?
    Eigen::VectorXd new_seed = seed;
    Eigen::VectorXd temp_solution(7);
    float min_dist_from_seed = __FLT_MAX__;
    bool found_solution = false;
    auto begin = std::chrono::steady_clock::now();
    float time_left;
    // cout << "Running IK: " << endl;
    while (true)
    {
        auto end = std::chrono::steady_clock::now();
        time_left = max_time_s - std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() * 1e-9;

        // No more time for another attempt
        if (time_left < 0) {
            // cout << "No more time left, stopping: " << time_left << " vs time_per_attempt: " << max_time_per_attempt_s << endl;
            break;
        }
        if (time_left < max_time_per_attempt_s) {
            ik_solver.get_mutable_prog()->SetSolverOption(solver_id,  "max_cpu_time", time_left);
        }

        // set seed
        ik_solver.get_mutable_prog()->SetInitialGuess(ik_solver.q().block<7,1>(arm_offset, 0), new_seed);

        // call solver and get solution
        // Eigen::VectorXd full_new_seed = Eigen::VectorXd::Zero(18);
        // full_new_seed.block<7,1>(arm_offset, 0) = new_seed;
        // std::optional<Eigen::VectorXd> new_seed_optional = full_new_seed;

        auto result = drake::solvers::Solve(ik_solver.prog(), solver_id_optional);
        // cout << "result and Cost: " << result.is_success() << ", " << result.get_optimal_cost() << endl;
        if (result.is_success()) {
            temp_solution = result.GetSolution().block<7,1>(arm_offset, 0);
            temp_solution = normalize_seed(seed, temp_solution, lower_limits, upper_limits);
            float dist_from_seed = (temp_solution - seed).squaredNorm();
            if (dist_from_seed < min_dist_from_seed) {
                min_dist_from_seed = dist_from_seed; 
                solved_joints = temp_solution;
                found_solution = true;
            }
        }

        // Try new_seed 
        for (unsigned int j=0; j<7; j++) {
            new_seed(j) = fRand(lower_limits(j), upper_limits(j));
        }
    }
    try {
        assert((lower_limits.array() <= solved_joints.array()).all());
        assert((solved_joints.array() <= upper_limits.array()).all());
    } catch (...) {
        cout << "Lower limits: " << endl;
        cout << lower_limits << endl;
        cout << "temp_solution" << endl;
        cout << temp_solution << endl;
    }
    cout << found_solution << endl;
    return found_solution;
}

// std::optional<lcmt_robot_plan> MoveIkDemoBase::Plan(
//     const math::RigidTransformd& goal_pose) {

//   DRAKE_THROW_UNLESS(status_count_ > 0);

//   // Create a single waypoint for our plan (the destination).
//   // This results in a trajectory with two knot points (the
//   // current pose (read from the status message currently being
//   // processes and passed directly to PlanSequentialTrajectory as
//   // iiwa_q) and the calculated final pose).
//   ConstraintRelaxingIk::IkCartesianWaypoint wp;
//   wp.pose = goal_pose;
//   wp.constrain_orientation = true;
//   std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
//   waypoints.push_back(wp);
//   std::vector<Eigen::VectorXd> q_sol;
//   const bool result =
//       constraint_relaxing_ik_.PlanSequentialTrajectory(
//           waypoints, plant_.GetPositions(*context_), &q_sol);
//   drake::log()->info("IK result: {}", result);

//   if (result) {
//     drake::log()->info("IK sol size {}", q_sol.size());

//     // Run the resulting plan over 2 seconds (which is a fairly
//     // arbitrary choice).  This may be slowed down if executing
//     // the plan in that time would exceed any joint velocity
//     // limits.
//     std::vector<double> times{0, 2};
//     DRAKE_DEMAND(q_sol.size() == times.size());

//     ApplyJointVelocityLimits(
//         q_sol, joint_velocity_limits_, &times);
//     lcmt_robot_plan plan = EncodeKeyFrames(
//         joint_names_, times, q_sol);
//     return plan;
//   }

//   return std::nullopt;
// }

bool check_collision_free(drake::multibody::MultibodyPlant<double>* plant,
                            drake::systems::Context<double>* plant_context,
                            drake::multibody::ModelInstanceIndex agent_idx,
                            const Eigen::VectorXd& joints,
                            float dist_threshold) {

    /*
    NOTE: Drake performs caching underneath for geometric evaluations, 
    and recommends re-creating new query ports and objects for every 
    geometric evaluation rather than keeping a single, persistent one throughout
    */
    plant->SetPositions(plant_context, agent_idx, joints);
    const auto &query_port = plant->get_geometry_query_input_port();
    const auto &query_object = query_port.Eval<drake::geometry::QueryObject<double>>(*plant_context);

    // OR IS THIS THE CORRECT USAGE?
    // const QueryObject<double>& query_object =
    // scene_graph.get_query_output_port().Eval<QueryObject<double>>(*context);

    std::vector<drake::geometry::SignedDistancePair<double>> signed_distance_pairs =
            query_object.ComputeSignedDistancePairwiseClosestPoints(0.1);
    bool is_collision_free = true;
    for (const auto &signed_distance_pair : signed_distance_pairs) {
        if (signed_distance_pair.distance < dist_threshold) {
            // uncomment to display info on the colliding object; warning, very verbose
            const auto& inspector = query_object.inspector(); const auto&
                    name_A = inspector.GetName(signed_distance_pair.id_A);
            const auto&
                    name_B = inspector.GetName(signed_distance_pair.id_B);
            drake::log()->info("{} <--> {} is: {}", name_A, name_B, signed_distance_pair.distance);
            is_collision_free = false;
        }
    }
    return is_collision_free;
}

std::optional<lcmt_robot_plan> MoveIkDemoBase::Plan(
    const math::RigidTransformd& goal_pose) {

  string sdf_filepath = "/home/armstrong/catkin_ws/src/vision/robot_meshes/shotwell_real_sidecar_robot_decomposed.sdf";
  float timestep = 0.0;
  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::MultibodyPlant<double>* plant{};
  drake::geometry::SceneGraph<double>* scene_graph{};
  std::tie(plant, scene_graph) = drake::multibody::AddMultibodyPlantSceneGraph(&builder, timestep);
  plant->set_name("plant");
  scene_graph->set_name("scene_graph");
  drake::multibody::Parser parser(plant, scene_graph);
  const auto robot_model_index = parser.AddModelFromFile(sdf_filepath, "robot");
  (void)robot_model_index;
  plant->Finalize();
  auto diagram = builder.Build();
  const auto source_id = plant->get_source_id().value();

  const auto& inspector = scene_graph->model_inspector();

  auto all_geom_ids = inspector.GetAllGeometryIds();

  // Exclude each arm's two fingers from collision with one another
  drake::geometry::GeometrySet left_arm_fingers_set, right_arm_fingers_set;

  // Ignore self-collisions of the other arm not being solved: assumes its current joint config is already valid
  drake::geometry::GeometrySet right_arm_set, left_arm_set;
  for (auto& geom_id : all_geom_ids) {
      const auto frame_id = inspector.GetFrameId(geom_id);
      const auto geom_model_index = inspector.GetFrameGroup(frame_id);

      std::string geom_name = inspector.GetName(geom_id);
      std::transform(geom_name.begin(), geom_name.end(), geom_name.begin(), ::tolower);

      if (geom_model_index == robot_model_index) {
          // if no "collision" in robot component (ie: visual), then ignore
          if (geom_name.find("collision") == std::string::npos) {
              scene_graph->RemoveRole(source_id, geom_id,
                  drake::geometry::Role::kProximity);
          }

          // Remove collision checking btwn each arm's fingers
          else if (geom_name.find("left_hande_left_finger_collision") != std::string::npos) {
              left_arm_fingers_set.Add(geom_id);
          } else if (geom_name.find("left_hande_right_finger_collision") != std::string::npos) {
              left_arm_fingers_set.Add(geom_id);
          } else if (geom_name.find("right_hande_left_finger_collision") != std::string::npos) {
              right_arm_fingers_set.Add(geom_id);
          } else if (geom_name.find("right_hande_right_finger_collision") != std::string::npos) {
              right_arm_fingers_set.Add(geom_id);
          } else if (
              (geom_name.find("right_hande") != std::string::npos) ||
              (geom_name.find("right_link") != std::string::npos)
          ){
              cout << "Ignoring " << geom_name << endl;
              right_arm_set.Add(geom_id);
          }
      }
  }
  
  scene_graph->collision_filter_manager().Apply(
      drake::geometry::CollisionFilterDeclaration()
          .ExcludeWithin(left_arm_fingers_set)
          .ExcludeWithin(right_arm_fingers_set)
          .ExcludeWithin(right_arm_set));
    

  // Only after all modifications to the plant/scene_graph have been made can we create a context with this updated info
  auto diagram_context= diagram->CreateDefaultContext();
  auto plant_context = &(diagram->GetMutableSubsystemContext(*plant,
  diagram_context.get()));

  std::vector<Eigen::VectorXd> q_sol;
  // q_sol.push_back(plant->GetPositions(*plant_context));
  cout << "init joints: " << endl;
  cout << plant->GetPositions(*plant_context).rows() << endl;
  cout << plant->GetPositions(*plant_context) << endl;

  Eigen::VectorXd target_position_in_robot = goal_pose.translation();
  Eigen::Matrix3d target_rot_mat_in_robot = goal_pose.rotation().matrix();
  target_rot_mat_in_robot << 0.240532,  -0.936105,  -0.256613,
 -0.157572,   0.223211,  -0.961949,
  0.957765,   0.271814, -0.0938143;
  cout << "trans: " << endl;
  cout << target_position_in_robot << endl;
  cout << "rot: " << endl;
  cout << target_rot_mat_in_robot << endl;
  Eigen::VectorXd init_joints(7);
  init_joints << 5.27906, 2.47798, 4.92036, 5.01675, 5.72805, 1.24125, 2.10627;

  float tol = __FLT_MAX__;
  Eigen::Vector3d position_buffer;
  position_buffer << 1e-3, 1e-3, 1e-3;
  float theta_bound = 1e-2;
  bool with_joint_limits = true;
  int max_iter = 500;
  float max_time_s = 0.1;  // DEFAULT_IK_TIMEOUT
  int min_attempts = 2;

  Eigen::VectorXd solved_joints(7), all_solved_joints(18);
  bool status = solve_ik(target_position_in_robot, 
            target_rot_mat_in_robot, plant, plant_context, init_joints, solved_joints, position_buffer, theta_bound, with_joint_limits, max_iter, max_time_s, min_attempts, tol);
  all_solved_joints.block<7,1>(0,0) = solved_joints;
  std::cout << "status" << endl;
  std::cout << status << endl;
  cout << "Solution" << endl;
  cout << solved_joints << endl;

  // Obtain the list of contacts.
  // const ContactResults<T>& contact_results =
  //     contact_results_input_port().template Eval<ContactResults<T>>(context);

  Eigen::VectorXd joints_config(18);
  joints_config.block<9,1>(0,0) << -5.25801, -0.911215, -5.60208, 2.4724, -3.7633, 2.48525, 3.95692, 0, 0;
  joints_config.block<9,1>(9,0) << -1.11401, -1.70838, -1.06991, 2.03128, 0.327641, 1.3965, -2.7309, 0, 0;

  check_collision_free(plant,
                        plant_context,
                        robot_model_index,
                        joints_config,
                        0.01);

  std::vector<double> times{0, 2};
  q_sol.push_back(joints_config);
  q_sol.push_back(joints_config);
  lcmt_robot_plan plan = EncodeKeyFrames(
      joint_names_, times, q_sol);
    return plan;
}

// std::optional<lcmt_robot_plan> MoveIkDemoBase::Plan(
//     const math::RigidTransformd& goal_pose) {
//   std::vector<Eigen::VectorXd> q_sol;
//   (void)goal_pose;
//   Eigen::VectorXd joints_config(18);
//   joints_config.block<9,1>(0,0) << 2.38427, 1.2675, 3.11826, 3.33899, -4.74074, 2.72956, 2.1227, 0, 0;
//   joints_config.block<9,1>(9,0) << -2.73018, -1.24598, 1.37487, 0.606355, -5.38772, -1.30804, -3.03414, 0, 0;
//   q_sol.push_back(joints_config);
//   q_sol.push_back(joints_config);
//   std::vector<double> times{0, 2};
//   lcmt_robot_plan plan = EncodeKeyFrames(
//       joint_names_, times, q_sol);
//     return plan;
// }

}  // namespace util
}  // namespace manipulation
}  // namespace drake
