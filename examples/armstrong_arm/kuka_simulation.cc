/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/armstrong_arm/iiwa_common.h"
#include "drake/examples/armstrong_arm/iiwa_lcm.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(sim_dt, 1e-5,
              "The time step to use for MultibodyPlant model "
              "discretization.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using multibody::MultibodyPlant;
using systems::Context;
using systems::Demultiplexer;
using systems::StateInterpolatorWithDiscreteDerivative;
using systems::Simulator;
using systems::controllers::InverseDynamicsController;
using systems::controllers::StateFeedbackControllerInterface;

int DoMain() {
    systems::DiagramBuilder<double> builder;

    // Adds a plant.
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(
        &builder, FLAGS_sim_dt);
    std::string sdf_filepath = "/home/armstrong/catkin_ws/src/vision/robot_meshes/shotwell_real_sidecar_robot_decomposed.sdf";
    auto iiwa_instance = multibody::Parser(
        &plant, &scene_graph).AddModelFromFile(sdf_filepath);
    plant.Finalize();

    // TODO(sammy-tri) Add a floor.

    // Creates and adds LCM publisher for visualization.
    auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    geometry::DrakeVisualizerParams params;
    params.role = geometry::Role::kProximity;
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, lcm, params);
    drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph, lcm);

    // Since we welded the model to the world above, the only remaining joints
    // should be those in the arm.
    const int num_joints = plant.num_positions();
    const int num_iiwa = 1;

    // Adds a iiwa controller.
    StateFeedbackControllerInterface<double>* controller = nullptr;
    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    iiwa_kp = iiwa_kp.replicate(num_iiwa, 1).eval();
    iiwa_kd = iiwa_kd.replicate(num_iiwa, 1).eval();
    iiwa_ki = iiwa_ki.replicate(num_iiwa, 1).eval();
    controller = builder.AddSystem<InverseDynamicsController<double>>(
        plant, iiwa_kp, iiwa_ki, iiwa_kd,
        false /* without feedforward acceleration */);

    // Create the command subscriber and status publisher.
    auto command_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
            "IIWA_COMMAND", lcm));
    command_sub->set_name("command_subscriber");
    auto command_receiver =
        builder.AddSystem<IiwaCommandReceiver>(num_joints);
    command_receiver->set_name("command_receiver");
    auto plant_state_demux = builder.AddSystem<Demultiplexer>(
        2 * num_joints, num_joints);
    plant_state_demux->set_name("plant_state_demux");
    auto desired_state_from_position = builder.AddSystem<
        StateInterpolatorWithDiscreteDerivative>(
            num_joints, kIiwaLcmStatusPeriod,
            true /* suppress_initial_transient */);
    desired_state_from_position->set_name("desired_state_from_position");
    auto status_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
            "IIWA_STATUS", lcm, kIiwaLcmStatusPeriod /* publish period */));
    status_pub->set_name("status_publisher");
    auto status_sender = builder.AddSystem<IiwaStatusSender>(num_joints);
    status_sender->set_name("status_sender");

    builder.Connect(command_sub->get_output_port(),
                    command_receiver->get_message_input_port());
    builder.Connect(plant_state_demux->get_output_port(0),
                    command_receiver->get_position_measured_input_port());
    builder.Connect(command_receiver->get_commanded_position_output_port(),
                    desired_state_from_position->get_input_port());
    builder.Connect(desired_state_from_position->get_output_port(),
                    controller->get_input_port_desired_state());
    builder.Connect(plant.get_state_output_port(iiwa_instance),
                    plant_state_demux->get_input_port(0));
    builder.Connect(plant_state_demux->get_output_port(0),
                    status_sender->get_position_measured_input_port());
    builder.Connect(plant_state_demux->get_output_port(1),
                    status_sender->get_velocity_estimated_input_port());
    builder.Connect(command_receiver->get_commanded_position_output_port(),
                    status_sender->get_position_commanded_input_port());
    builder.Connect(plant.get_state_output_port(),
                    controller->get_input_port_estimated_state());
    builder.Connect(controller->get_output_port_control(),
                    plant.get_actuation_input_port(iiwa_instance));
    builder.Connect(controller->get_output_port_control(),
                    status_sender->get_torque_commanded_input_port());
    builder.Connect(controller->get_output_port_control(),
                    status_sender->get_torque_measured_input_port());
    // TODO(sammy-tri) Add a low-pass filter for simulated external torques.
    // This would slow the simulation significantly, however.  (see #12631)
    builder.Connect(
        plant.get_generalized_contact_forces_output_port(iiwa_instance),
        status_sender->get_torque_external_input_port());
    builder.Connect(status_sender->get_output_port(),
                    status_pub->get_input_port());

    auto sys = builder.Build();

    Simulator<double> simulator(*sys);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();

    // Simulate for a very long time.
    simulator.AdvanceTo(FLAGS_simulation_sec);

    return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::kuka_iiwa_arm::DoMain();
}
