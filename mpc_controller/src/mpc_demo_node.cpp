#include "mpc_controller/mpc.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
  using MPC = mpc_controller::MPCController;
  using Vector = MPC::Vector;

  MPC::Options options;
  options.prediction_horizon = 20;
  options.control_horizon = 8;
  options.dt = 0.1;
  options.max_iterations = 25;
  options.initial_step_size = 0.5;
  options.step_decay = 0.55;
  options.min_step_size = 1e-4;
  options.warm_start = true;

  MPC solver(options);

  const Vector target = (Vector(2) << 3.0, 0.0).finished();
  const Eigen::Matrix2d q = (Eigen::Matrix2d() << 8.0, 0.0, 0.0, 1.0).finished();
  const Eigen::Matrix<double, 1, 1> r =
    (Eigen::Matrix<double, 1, 1>() << 0.05).finished();
  const Eigen::Matrix2d q_terminal =
    (Eigen::Matrix2d() << 25.0, 0.0, 0.0, 5.0).finished();

  auto dynamics = [](const Vector& x, const Vector& u, double dt, int) {
    Eigen::Matrix2d a;
    a << 1.0, dt,
         0.0, 1.0;

    Eigen::Matrix<double, 2, 1> b;
    b << 0.5 * dt * dt,
         dt;

    return Vector(a * x + b * u(0));
  };

  auto stage_cost = [target, q, r](const Vector& x, const Vector& u, int) {
    const Vector error = x - target;
    return (error.transpose() * q * error)(0, 0) +
      (u.transpose() * r * u)(0, 0);
  };

  auto terminal_cost = [target, q_terminal](const Vector& x) {
    const Vector error = x - target;
    return (error.transpose() * q_terminal * error)(0, 0);
  };

  Vector state = (Vector(2) << 0.0, 0.0).finished();

  std::cout << "Generic MPC demo: double integrator position control\n";
  std::cout << "state = [position, velocity], control = [acceleration]\n\n";
  std::cout << std::setw(5) << "step"
            << std::setw(12) << "position"
            << std::setw(12) << "velocity"
            << std::setw(14) << "accel_cmd"
            << std::setw(14) << "cost"
            << '\n';

  for (int step = 0; step < 50; ++step) {
    MPC::Problem problem;
    problem.initial_state = state;
    problem.nominal_control = Vector::Zero(1);
    problem.control_lower_bound = Vector::Constant(1, -2.0);
    problem.control_upper_bound = Vector::Constant(1, 2.0);
    problem.dynamics = dynamics;
    problem.stage_cost = stage_cost;
    problem.terminal_cost = terminal_cost;

    const MPC::Result result = solver.solve(problem);
    const Vector control = result.firstControl();

    std::cout << std::setw(5) << step
              << std::setw(12) << std::fixed << std::setprecision(4) << state(0)
              << std::setw(12) << std::fixed << std::setprecision(4) << state(1)
              << std::setw(14) << std::fixed << std::setprecision(4) << control(0)
              << std::setw(14) << std::fixed << std::setprecision(4) << result.cost
              << '\n';

    state = dynamics(state, control, options.dt, step);

    if ((state - target).norm() < 0.05) {
      std::cout << "\nTarget reached within tolerance.\n";
      break;
    }
  }

  std::cout << "\nFinal state: [" << state(0) << ", " << state(1) << "]\n";
  return 0;
}
