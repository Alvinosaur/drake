#include "drake/solvers/solve.h"

#include <memory>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<SolverId>& solver_id_input,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
    
  std::unique_ptr<SolverId> solver_id;
  if (solver_id_input) {
    solver_id = std::make_unique<SolverId>(solver_id_input.value());
  } else {
    solver_id = std::make_unique<SolverId>(ChooseBestSolver(prog));
  }
  drake::log()->debug("solvers::Solve will use {}", *solver_id.get());
  std::unique_ptr<SolverInterface> solver = MakeSolver(*solver_id.get());
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<SolverId>& solver_id_input) {
  return Solve(prog, solver_id_input, {}, {});
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<SolverId>& solver_id_input,
    const std::optional<Eigen::VectorXd>& initial_guess) {
  return Solve(prog, solver_id_input, initial_guess, {});
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  return Solve(prog, {}, initial_guess, solver_options);
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
  const Eigen::VectorXd initial_guess_xd = initial_guess;
  return Solve(prog, {}, initial_guess_xd, {});
}

MathematicalProgramResult Solve(const MathematicalProgram& prog) {
  return Solve(prog, {}, {}, {});
}
}  // namespace solvers
}  // namespace drake
