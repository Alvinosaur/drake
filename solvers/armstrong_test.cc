#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/solve.h"
#include "drake/multibody/parsing/parser.h"

// #include "drake/solvers/clp_solver.h"
// #include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/nlopt_solver.h"

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
// #include "drake/multibody/tree/multibody_tree_indexes.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/geometry/drake_visualizer.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <fstream>
#include <chrono>
#include <numeric>
#include <bits/stdc++.h>

// using namespace boost::filesystem;    

using namespace std;

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
    float min_distance = 0.0;  // padding distance
    float influence_distance_offset = 1;  // TODO: not sure what this parameter does...
    ik_solver.AddMinimumDistanceConstraint(min_distance, influence_distance_offset);

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

void read_csv(string fpath, vector<vector<double>> &data)
{
    std::fstream file(fpath, ios::in);
    if(!file.is_open())
    {
           cout << "File not found: " << fpath << endl;
           exit(1);
    }

    // read every line from the stream
    string csvLine;
    while( getline(file, csvLine) )
    {
        istringstream csvStream(csvLine);
        vector<double> row;
        string csvElement;
        // read every element from the line that is seperated by commas
        // and put it into the vector or strings
        while( getline(csvStream, csvElement, ',') )
        {
            row.emplace_back(std::stod(csvElement));
        }
        data.emplace_back(move(row));
    }
    cout << "Successfully READ file: " << fpath << endl;
    file.close();
}

void write_csv(string fpath, const vector<vector<double>> &data) {
    std::ofstream file;
    file.open(fpath);
    for(auto& row : data)
    {
        int size = row.size();
        if (size == 0) {
            file << endl;
            continue;
        }
        for (int i=0; i<size-1; ++i) {
            file << row[i] << ", ";
        }
        file << row[size-1];
        file << endl;
    }
    file.close();
    cout << "Successfully WROTE file: " << fpath << endl;
}

void write_csv(string fpath, const vector<Eigen::VectorXd> &data) {
    std::ofstream file;
    file.open(fpath);
    for(auto& row : data)
    {
        int size = row.rows();
        if (size == 0) {
            file << endl;
            continue;
        }
        for (int i=0; i<size-1; ++i) {
            file << row(i) << ", ";
        }
        file << row(size-1);
        file << endl;
    }
    file.close();
    cout << "Successfully WROTE file: " << fpath << endl;
}

void write_csv(string fpath, const vector<double> &data) {
    std::ofstream file;
    file.open(fpath);
    for(auto& val : data)
    {
        file << val;
        file << endl;
    }
    file.close();
    cout << "Successfully WROTE file: " << fpath << endl;
}

void write_csv(string fpath, const vector<bool> &data) {
    std::ofstream file;
    file.open(fpath);
    for(auto val : data)
    {
        file << val;
        file << endl;
    }
    file.close();
    cout << "Successfully WROTE file: " << fpath << endl;
}

void write_csv(string fpath, const vector<vector<int>> &data) {
    std::ofstream file;
    file.open(fpath);
    for(auto& row : data)
    {
        int size = row.size();
        if (size == 0) {
            file << endl;
            continue;
        }
        for (int i=0; i<size-1; ++i) {
            file << row[i] << ", ";
        }
        file << row[size-1];
        file << endl;
    }
    file.close();
    cout << "Successfully WROTE file: " << fpath << endl;
}


/*

    Usage: bazel run --config gurobi --config snopt //solvers:armstrong_test

*/
int main() {
    string sdf_filepath = "/home/armstrong/catkin_ws/src/vision/custom_urdfs_sdfs/shotwell_real_sidecar_NEW.sdf";

    float timestep = 0.0;
    bool with_joint_limits = true;
    int max_iter = 500;
    float max_time_s = 0.1;  // DEFAULT_IK_TIMEOUT
    int min_attempts = 2;

    // see arm.cpp:5043 epsilon
    float tol = __FLT_MAX__;
    Eigen::Vector3d position_buffer;
    position_buffer << 1e-3, 1e-3, 1e-3;
    float theta_bound = 1e-2;

    drake::systems::DiagramBuilder<double> builder;
    drake::multibody::MultibodyPlant<double>* plant{};
    drake::geometry::SceneGraph<double>* scene_graph{};
    std::tie(plant, scene_graph) = drake::multibody::AddMultibodyPlantSceneGraph(&builder, timestep);
    plant->set_name("plant");
    scene_graph->set_name("scene_graph");
    drake::multibody::Parser parser(plant, scene_graph);

    // Create an obstacle 
    bool add_obstacles = false;
    if (add_obstacles) {
        float radius = 1.0;
        float mass = 0.1;
        const Eigen::Vector3d p_BoBcm = Eigen::Vector3d::Zero();
        const drake::multibody::RigidBody<double>& ball = plant->AddRigidBody(
            "Ball", drake::multibody::SpatialInertia<double>{
                mass, p_BoBcm, drake::multibody::UnitInertia<double>::SolidSphere(radius)});

        // Set up mechanical properties of the ball.
        drake::geometry::ProximityProperties ball_props;
        double dummy_dissipation = 0.1;
        double dummy_friction = 0.1;  
        double dummy_hydroelastic_modulus = 0.1;
        drake::multibody::CoulombFriction dummy_surface_friction(dummy_friction, dummy_friction);
        AddContactMaterial(dummy_dissipation, {} /* point stiffness */, dummy_surface_friction,
                     &ball_props);
        AddCompliantHydroelasticProperties(radius,
                                            dummy_hydroelastic_modulus, &ball_props);

        auto geometry_in_body = drake::math::RigidTransformd::Identity();
        plant->RegisterCollisionGeometry(ball, geometry_in_body,
                                        drake::geometry::Sphere(radius), "collision",
                                        std::move(ball_props));

        Eigen::VectorXd orange(4);
        orange << 1.0, 0.55, 0.0, 0.2;
        plant->RegisterVisualGeometry(ball, drake::math::RigidTransformd::Identity(),
                                drake::geometry::Sphere(radius), "visual", orange);

        // fix ball's pose
        Eigen::Vector3d ball_position;
        Eigen::Matrix3d ball_rotation;
        ball_position << 0.0950948, 0.183158, -0.0956849;
        plant->WeldFrames(plant->world_frame(),
                        plant->GetFrameByName("Ball"));
        assert (false); // TODO: how to set pose of obstacle? How to update pose continously?

    }

    // NOTE: must add robot after all objects?? Otherwise get error:
    // This model has more model instances than the default.  Please call AddRigidBody with an explicit model instance.
    const auto robot_model_index = parser.AddModelFromFile(sdf_filepath, "robot");
    // parser.AddModelFromFile(sdf_filepath, "robot");
    // const auto& robot_ {robot_model_index.get_source_id().value()};
    // plant->WeldFrames(plant->world_frame(),
    //                        plant->GetFrameByName("robot"));

    // Connect plant with scene_graph to get collision information.
    // builder.Connect(
    //     plant->get_geometry_poses_output_port(),
    //     scene_graph->get_source_pose_port(plant->get_source_id().value()));
    // builder.Connect(scene_graph->get_query_output_port(),
    //                 plant->get_geometry_query_input_port());

    // plant->RegisterAsSourceForSceneGraph(scene_graph);

    plant->Finalize();
    
    auto diagram = builder.Build();
    auto diagram_context= diagram->CreateDefaultContext();
    auto plant_context = &(diagram->GetMutableSubsystemContext(*plant,
    diagram_context.get()));

    // External and self collision constraints
    // float min_distance = 0.0;  // padding distance
    // ik_solver.AddMinimumDistanceConstraint(min_distance);

    // scene graph inspector only useful for static info NOT dependent on pose
    const auto& inspector = scene_graph->model_inspector();
    // const auto query_port = scene_graph->get_query_output_port();
    // const auto& query_object {
    //   query_port.Eval<drake::geometry::QueryObject<double>>(*plant_context)};
    // const auto& inspector {query_object.inspector()};
    // const auto& collision_geometries = plant.GetCollisionGeometriesForBody(some_body);
    // https://stackoverflow.com/questions/68225198/does-a-modelinstance-in-a-multibodyplan-have-a-unique-source-id-for-determining
    // const auto& source_id = plant->get_source_id().value();
    // cout << "Source id: " << source_id << endl;    

    auto all_geom_ids = inspector.GetAllGeometryIds();
    for (auto& geom_id : all_geom_ids) {
        // if geometry belongs to robot and contains "collision" which is how
        // we define collision bodies in urdf/srdf
        const auto frame_id = inspector.GetFrameId(geom_id);
        const auto geom_model_index = inspector.GetFrameGroup(frame_id);
        const auto source_name = inspector.GetOwningSourceName(frame_id);
        const auto source_id = inspector.GetOwningSourceId(geom_id);
        std::string geom_name = inspector.GetName(geom_id);
        std::transform(geom_name.begin(), geom_name.end(), geom_name.begin(), ::tolower);
        // cout << "frame_id: " << frame_id << endl;
        // cout << "geom_model_index: " << geom_model_index << endl;
        // cout << "robot_model_index: " << robot_model_index << endl;
        // cout << "name: " << geom_name << endl;
        // cout << "source_name: " << source_name << endl;
        // cout << "source_id: " << source_id << endl;
        // cout << endl;
        
        if (geom_model_index == robot_model_index && geom_name.find("collision") == std::string::npos) {
            cout << "Removing Proximity of: " << geom_name << endl;
            scene_graph->RemoveRole(source_id, geom_id,
                drake::geometry::Role::kProximity);
            // scene_graph->AssignRole(source_id, geometry_id, ProximityProperties());
            // geom->SetRole()
        }
    }
    auto collision_pairs = inspector.GetCollisionCandidates();
    cout << "Collision pairs: " << endl;
    for (const auto& [first_geom_id, second_geom_id] : collision_pairs) {
        const auto first_name = inspector.GetName(first_geom_id); 
        const auto second_name = inspector.GetName(second_geom_id); 
        cout << first_name << ", " << second_name << endl;

        cout << "Left: " << endl;
        const auto first_frame_id = inspector.GetFrameId(first_geom_id);
        const auto first_geom_model_index = inspector.GetFrameGroup(first_frame_id);
        const auto first_source_name = inspector.GetOwningSourceName(first_frame_id);
        const auto first_source_id = inspector.GetOwningSourceId(first_geom_id);
        cout << "frame_id: " << first_frame_id << endl;
        cout << "geom_model_index: " << first_geom_model_index << endl;
        cout << "robot_model_index: " << robot_model_index << endl;
        cout << "source_name: " << first_source_name << endl;
        cout << "source_id: " << first_source_id << endl;
        cout << endl;

        cout << "Right: " << endl;
        const auto second_frame_id = inspector.GetFrameId(second_geom_id);
        const auto second_geom_model_index = inspector.GetFrameGroup(second_frame_id);
        const auto second_source_name = inspector.GetOwningSourceName(second_frame_id);
        const auto second_source_id = inspector.GetOwningSourceId(second_geom_id);
        cout << "frame_id: " << second_frame_id << endl;
        cout << "geom_model_index: " << second_geom_model_index << endl;
        cout << "robot_model_index: " << robot_model_index << endl;
        cout << "source_name: " << second_source_name << endl;
        cout << "source_id: " << second_source_id << endl;
        cout << endl;
    }
    
    // Useful for future collision filtering as desired:
    // https://stackoverflow.com/questions/66285704/how-to-define-collision-filter-groups-across-model-instances

    // QueryObject useful for checking collisions

    // SceneGraph provides the interface for registering the geometry, updating its position based on the current context, and performing geometric queries.
    // specifically use GeometryConfigurationVector or FramePoseVector to indirectly set KinematicsVector(internal)
    // GeometryConfigurationVector used to update mesh vertices of deformable objects
    // FramePoseVector used to update kinematic poses
    // Or should it be context??? Dynamic geometry can move; more specifically, its kinematics (e.g., pose) depends on a system's Context. Particularly, a non-deformable dynamic geometry is fixed to a frame whose kinematics values depend on a context. As the frame moves, the geometries fixed to it move with it.

    vector<vector<double>> all_ee_poses, all_init_joints;
    read_csv("/home/armstrong/catkin_ws/src/vision/rand_ee_poses.txt", all_ee_poses);
    read_csv("/home/armstrong/catkin_ws/src/vision/rand_joints.txt", all_init_joints);
    all_ee_poses.resize(20);
    all_init_joints.resize(5);

    const int total_num_trials = all_ee_poses.size() * all_init_joints.size();
    vector<vector<double>> all_times;
    vector<vector<int>> all_statuses;
    vector<Eigen::VectorXd> all_results;
    vector<vector<double>> all_dists_from_seed;
    all_times.reserve(all_ee_poses.size());
    all_results.reserve(total_num_trials);
    bool status;
    vector<double> solved_joints_vec(7, 0);
    Eigen::VectorXd solved_joints(7), init_joints(7);
    Eigen::Vector3d target_position_in_robot;
    Eigen::Quaterniond target_rot_quat_in_robot;
    Eigen::Matrix3d target_rot_mat_in_robot;
    Eigen::VectorXd ee_trans_quat(7);

    // target_position_in_robot << 0.0950948, 0.183158, -0.0956849;
    // target_rot_mat_in_robot <<  0.258104,  0.665297,  0.700544,
    //                             -0.663628, -0.404887,  0.629018,
    //                             0.702125, -0.627253,  0.337007;
    // status = solve_ik(target_position_in_robot, 
    //     target_rot_mat_in_robot, plant, solved_joints, position_buffer, theta_bound, with_joint_limits, &init_joints);      
    // cout << status << endl;
    // cout << solved_joints << endl;
    // exit(0);             

    for (auto& ee_trans_quat_vec : all_ee_poses) {
        ee_trans_quat = Eigen::Map<Eigen::Matrix<double, 7, 1>>(ee_trans_quat_vec.data());
        target_position_in_robot = ee_trans_quat.block<3,1>(0,0);
        target_rot_quat_in_robot.x() = ee_trans_quat(3);
        target_rot_quat_in_robot.y() = ee_trans_quat(4);
        target_rot_quat_in_robot.z() = ee_trans_quat(5);
        target_rot_quat_in_robot.w() = ee_trans_quat(6);
        target_rot_mat_in_robot = target_rot_quat_in_robot.toRotationMatrix();

        vector<double> local_times;
        vector<int> local_statuses;
        vector<double> local_dists_from_seed;

        for (auto& init_joints_vec : all_init_joints) {
            init_joints = Eigen::Map<Eigen::Matrix<double, 7, 1> >(init_joints_vec.data());

            auto begin = std::chrono::steady_clock::now();

            status = solve_ik(target_position_in_robot, 
            target_rot_mat_in_robot, plant, plant_context, init_joints, solved_joints, position_buffer, theta_bound, with_joint_limits, max_iter, max_time_s, min_attempts, tol);

            auto end = std::chrono::steady_clock::now();

            // pick solution closest to seed

            local_times.push_back(
                std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() * 1e-9
            );
            local_statuses.push_back(static_cast<int>(status));

            // if (status){ 
            // for (int i=0; i<7; i++) solved_joints_vec[i] = solved_joints(i);
            if (status) {
                all_results.push_back(solved_joints);
                local_dists_from_seed.push_back((solved_joints - init_joints).cwiseAbs().mean());
            } 
        }

        all_times.emplace_back(move(local_times));
        all_statuses.emplace_back(move(local_statuses)); // ee_poses x init_joints
        all_dists_from_seed.emplace_back(move(local_dists_from_seed));  // ee_poses x (NOTE) num_successes
    }

    // Save raw results
    write_csv("/home/armstrong/test_ws/src/simple_test/drake_runtimes.txt", all_times);
    write_csv("/home/armstrong/test_ws/src/simple_test/drake_solutions.txt", all_results);
    write_csv("/home/armstrong/test_ws/src/simple_test/drake_statuses.txt", all_statuses);
    write_csv("/home/armstrong/test_ws/src/simple_test/drake_dists.txt", all_dists_from_seed);
    
    // Run evaluation
    // min, max, avg, stddev distance of solution from seed
    
    int overall_total_success = 0;
    float overall_total_dist = 0.0;
    float overall_total_sq_dist = 0.0;
    for (size_t ti=0; ti<all_ee_poses.size(); ti++) {
        int total_success = std::accumulate(
                all_statuses[ti].begin(), 
                all_statuses[ti].end(), 0.0);
        float total_dist_from_seed = std::accumulate(
                all_dists_from_seed[ti].begin(), 
                all_dists_from_seed[ti].end(), 0.0);
        float mean = total_dist_from_seed / static_cast<float>(total_success);
        float total_sq_dist_from_seed = std::accumulate(
            all_dists_from_seed[ti].begin(), 
            all_dists_from_seed[ti].end(), 0.0,
            [mean](float sum, float x){ return sum + pow(x-mean, 2); }
        );
        float stddev_dist = std::sqrt(total_sq_dist_from_seed / static_cast<float>(total_success));

        cout << "local target " << ti << " metrics: " << endl;
        cout << "success rate: " << static_cast<float>(total_success) / static_cast<float>(all_statuses[ti].size()) << endl;
        cout << "avg dist: " << total_dist_from_seed / static_cast<float>(total_success) << endl;
        cout << "stddev dist: " << stddev_dist << endl;

        overall_total_success += total_success;
        overall_total_dist += total_dist_from_seed;
        overall_total_sq_dist += total_sq_dist_from_seed;
    }

    cout << endl;
    cout << "OVERALL metrics: " << endl;
    cout << "success rate: " << static_cast<float>(overall_total_success) / static_cast<float>(total_num_trials) << endl;
    cout << "avg dist: " << overall_total_dist / static_cast<float>(overall_total_success) << endl;
    float overall_stddev_dist = std::sqrt(overall_total_sq_dist / static_cast<float>(overall_total_success));
    cout << "stddev dist: " << overall_stddev_dist << endl;

}
