#ifndef GENERIC_MPC_CONTROLLER_HPP
#define GENERIC_MPC_CONTROLLER_HPP

#include <eigen3/Eigen/Dense>

#include <functional>
#include <string>
#include <vector>

namespace mpc_controller
{

class MPCController
{
public:
  using Vector = Eigen::VectorXd;
  using Trajectory = std::vector<Vector>;
  using ControlSequence = std::vector<Vector>;
  using DynamicsFunction =
    std::function<Vector(const Vector& state, const Vector& control, double dt, int step)>;
  using StageCostFunction =
    std::function<double(const Vector& state, const Vector& control, int step)>;
  using TerminalCostFunction = std::function<double(const Vector& state)>;

  struct Options
  {
    int prediction_horizon = 10;
    int control_horizon = 0;
    double dt = 0.1;
    int max_iterations = 20;
    double initial_step_size = 1.0;
    double step_decay = 0.5;
    double min_step_size = 1e-4;
    bool warm_start = true;
  };

  struct Problem
  {
    Vector initial_state;
    Vector nominal_control;
    Vector control_lower_bound;
    Vector control_upper_bound;
    DynamicsFunction dynamics;
    StageCostFunction stage_cost;
    TerminalCostFunction terminal_cost;
    ControlSequence initial_controls;
  };

  struct Result
  {
    bool success = false;
    double cost = 0.0;
    int iterations = 0;
    std::string message;
    Trajectory states;
    ControlSequence controls;

    const Vector& firstControl() const;
  };

  explicit MPCController(const Options& options);
  ~MPCController() = default;

  Result solve(const Problem& problem);
  void resetWarmStart();

  const Options& options() const;
  const ControlSequence& lastOptimizedControls() const;

private:
  int predictionHorizon() const;
  int controlHorizon() const;

  void validateProblem(const Problem& problem) const;
  ControlSequence makeInitialControls(const Problem& problem) const;
  ControlSequence expandControls(const ControlSequence& decision_controls) const;
  Vector clampControl(const Vector& control, const Problem& problem) const;

  double evaluateCost(
    const Problem& problem, const ControlSequence& decision_controls,
    Trajectory* states = nullptr) const;

  Options options_;
  ControlSequence warm_start_controls_;
};

} // namespace mpc_controller

#endif // GENERIC_MPC_CONTROLLER_HPP
