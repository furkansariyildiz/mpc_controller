#include "mpc_controller/mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mpc_controller
{

const MPCController::Vector& MPCController::Result::firstControl() const
{
  if (controls.empty()) {
    throw std::runtime_error("MPC result has no controls");
  }

  return controls.front();
}

MPCController::MPCController(const Options& options)
  : options_(options)
{
  if (options_.prediction_horizon < 1) {
    options_.prediction_horizon = 1;
  }

  if (options_.control_horizon < 1 ||
      options_.control_horizon > options_.prediction_horizon) {
    options_.control_horizon = options_.prediction_horizon;
  }

  if (options_.dt <= 0.0) {
    options_.dt = 0.1;
  }

  if (options_.max_iterations < 1) {
    options_.max_iterations = 1;
  }

  if (options_.initial_step_size <= 0.0) {
    options_.initial_step_size = 1.0;
  }

  if (options_.step_decay <= 0.0 || options_.step_decay >= 1.0) {
    options_.step_decay = 0.5;
  }

  if (options_.min_step_size <= 0.0) {
    options_.min_step_size = 1e-4;
  }
}

MPCController::Result MPCController::solve(const Problem& problem)
{
  validateProblem(problem);

  ControlSequence decision_controls = makeInitialControls(problem);
  double best_cost = evaluateCost(problem, decision_controls);
  double step_size = options_.initial_step_size;
  int completed_iterations = 0;

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    bool improved = false;

    for (int control_index = 0; control_index < controlHorizon(); ++control_index) {
      for (int component = 0; component < problem.nominal_control.size(); ++component) {
        const double original_value = decision_controls[control_index](component);
        double best_value = original_value;
        double local_best_cost = best_cost;

        for (const double direction : {-1.0, 1.0}) {
          ControlSequence candidate = decision_controls;
          candidate[control_index](component) = original_value + direction * step_size;
          candidate[control_index] = clampControl(candidate[control_index], problem);

          const double candidate_cost = evaluateCost(problem, candidate);
          if (candidate_cost < local_best_cost) {
            local_best_cost = candidate_cost;
            best_value = candidate[control_index](component);
          }
        }

        if (local_best_cost + 1e-12 < best_cost) {
          decision_controls[control_index](component) = best_value;
          best_cost = local_best_cost;
          improved = true;
        }
      }
    }

    ++completed_iterations;
    step_size *= improved ? options_.step_decay : options_.step_decay * 0.5;
    if (step_size < options_.min_step_size) {
      break;
    }
  }

  Trajectory states;
  const double final_cost = evaluateCost(problem, decision_controls, &states);
  ControlSequence expanded_controls = expandControls(decision_controls);

  if (options_.warm_start) {
    warm_start_controls_ = decision_controls;
    if (warm_start_controls_.size() > 1) {
      for (size_t i = 0; i + 1 < warm_start_controls_.size(); ++i) {
        warm_start_controls_[i] = warm_start_controls_[i + 1];
      }
    }
  }

  Result result;
  result.success = true;
  result.cost = final_cost;
  result.iterations = completed_iterations;
  result.message = "Solved with coordinate search";
  result.states = states;
  result.controls = expanded_controls;
  return result;
}

void MPCController::resetWarmStart()
{
  warm_start_controls_.clear();
}

const MPCController::Options& MPCController::options() const
{
  return options_;
}

const MPCController::ControlSequence& MPCController::lastOptimizedControls() const
{
  return warm_start_controls_;
}

int MPCController::predictionHorizon() const
{
  return options_.prediction_horizon;
}

int MPCController::controlHorizon() const
{
  return options_.control_horizon;
}

void MPCController::validateProblem(const Problem& problem) const
{
  if (problem.initial_state.size() == 0) {
    throw std::invalid_argument("MPC problem requires a non-empty initial_state");
  }

  if (problem.nominal_control.size() == 0) {
    throw std::invalid_argument("MPC problem requires a non-empty nominal_control");
  }

  if (!problem.dynamics) {
    throw std::invalid_argument("MPC problem requires a dynamics function");
  }

  if (!problem.stage_cost) {
    throw std::invalid_argument("MPC problem requires a stage_cost function");
  }

  if (problem.control_lower_bound.size() != 0 &&
      problem.control_lower_bound.size() != problem.nominal_control.size()) {
    throw std::invalid_argument("control_lower_bound size must match nominal_control size");
  }

  if (problem.control_upper_bound.size() != 0 &&
      problem.control_upper_bound.size() != problem.nominal_control.size()) {
    throw std::invalid_argument("control_upper_bound size must match nominal_control size");
  }

  if (problem.control_lower_bound.size() != 0 &&
      problem.control_upper_bound.size() != 0) {
    for (int i = 0; i < problem.nominal_control.size(); ++i) {
      if (problem.control_lower_bound(i) > problem.control_upper_bound(i)) {
        throw std::invalid_argument("control lower bound cannot be greater than upper bound");
      }
    }
  }
}

MPCController::ControlSequence MPCController::makeInitialControls(
  const Problem& problem) const
{
  if (problem.initial_controls.size() == static_cast<size_t>(controlHorizon())) {
    ControlSequence controls = problem.initial_controls;
    for (Vector& control : controls) {
      if (control.size() != problem.nominal_control.size()) {
        throw std::invalid_argument("initial_controls entries must match nominal_control size");
      }
      control = clampControl(control, problem);
    }
    return controls;
  }

  if (options_.warm_start &&
      warm_start_controls_.size() == static_cast<size_t>(controlHorizon())) {
    ControlSequence controls = warm_start_controls_;
    for (Vector& control : controls) {
      if (control.size() == problem.nominal_control.size()) {
        control = clampControl(control, problem);
      } else {
        controls.clear();
        break;
      }
    }

    if (!controls.empty()) {
      return controls;
    }
  }

  ControlSequence controls;
  controls.reserve(static_cast<size_t>(controlHorizon()));
  const Vector nominal_control = clampControl(problem.nominal_control, problem);
  for (int i = 0; i < controlHorizon(); ++i) {
    controls.push_back(nominal_control);
  }
  return controls;
}

MPCController::ControlSequence MPCController::expandControls(
  const ControlSequence& decision_controls) const
{
  ControlSequence expanded_controls;
  expanded_controls.reserve(static_cast<size_t>(predictionHorizon()));

  for (int k = 0; k < predictionHorizon(); ++k) {
    const int index = std::min(k, controlHorizon() - 1);
    expanded_controls.push_back(decision_controls[static_cast<size_t>(index)]);
  }

  return expanded_controls;
}

MPCController::Vector MPCController::clampControl(
  const Vector& control, const Problem& problem) const
{
  Vector clamped = control;

  for (int i = 0; i < clamped.size(); ++i) {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();

    if (problem.control_lower_bound.size() == clamped.size()) {
      lower = problem.control_lower_bound(i);
    }

    if (problem.control_upper_bound.size() == clamped.size()) {
      upper = problem.control_upper_bound(i);
    }

    clamped(i) = std::max(lower, std::min(upper, clamped(i)));
  }

  return clamped;
}

double MPCController::evaluateCost(
  const Problem& problem, const ControlSequence& decision_controls,
  Trajectory* states) const
{
  if (decision_controls.size() != static_cast<size_t>(controlHorizon())) {
    throw std::invalid_argument("decision control sequence size does not match control horizon");
  }

  Vector state = problem.initial_state;
  double total_cost = 0.0;

  if (states != nullptr) {
    states->clear();
    states->reserve(static_cast<size_t>(predictionHorizon() + 1));
    states->push_back(state);
  }

  for (int k = 0; k < predictionHorizon(); ++k) {
    const int control_index = std::min(k, controlHorizon() - 1);
    const Vector& control = decision_controls[static_cast<size_t>(control_index)];

    total_cost += problem.stage_cost(state, control, k);
    state = problem.dynamics(state, control, options_.dt, k);

    if (state.size() != problem.initial_state.size()) {
      throw std::runtime_error("dynamics returned a state with an unexpected size");
    }

    if (states != nullptr) {
      states->push_back(state);
    }
  }

  if (problem.terminal_cost) {
    total_cost += problem.terminal_cost(state);
  }

  return total_cost;
}

} // namespace mpc_controller
