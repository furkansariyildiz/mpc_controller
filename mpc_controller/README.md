# mpc_controller

`mpc_controller` is a generic native Model Predictive Control solver packaged as a ROS2 ament library (Humble). It is not tied to a vehicle, lateral control, robotics, or any specific physical model.

You provide:

- the current state,
- a nominal control vector,
- optional control bounds,
- a dynamics function,
- a stage cost function,
- optionally a terminal cost function.

The solver returns:

- the predicted state trajectory,
- the optimized control sequence,
- the first control to apply,
- the final trajectory cost.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mpc_controller
source install/setup.bash
```

## Run The Demo

The demo solves a simple double-integrator position-control problem:

```text
state   = [position, velocity]
control = [acceleration]
```

Run:

```bash
ros2 run mpc_controller mpc_demo_node
```

The executable prints the receding-horizon control loop: current position, velocity, selected acceleration command, and MPC cost.

## Basic Usage

Include the header:

```cpp
#include "mpc_controller/mpc.hpp"
```

Create solver options:

```cpp
using MPC = mpc_controller::MPCController;

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
```

Define a problem:

```cpp
MPC::Problem problem;
problem.initial_state = x0;
problem.nominal_control = u_nominal;
problem.control_lower_bound = u_min;
problem.control_upper_bound = u_max;
problem.dynamics = dynamics_fn;
problem.stage_cost = stage_cost_fn;
problem.terminal_cost = terminal_cost_fn;
```

Solve it:

```cpp
const MPC::Result result = solver.solve(problem);

if (result.success) {
  const Eigen::VectorXd& u0 = result.firstControl();
}
```

## Dynamics Function

The dynamics function maps a state and control to the next state:

```cpp
problem.dynamics =
  [](const Eigen::VectorXd& x,
     const Eigen::VectorXd& u,
     double dt,
     int step) {
    Eigen::VectorXd x_next = x;
    // Fill in your model here.
    return x_next;
  };
```

The model can be any discrete-time update:

```text
x[k+1] = f(x[k], u[k], dt, k)
```

If your physical model is continuous-time, discretize it inside this callback with Euler, RK4, or any other method you prefer.

## Cost Functions

The stage cost is evaluated at every prediction step:

```cpp
problem.stage_cost =
  [](const Eigen::VectorXd& x,
     const Eigen::VectorXd& u,
     int step) {
    return x.squaredNorm() + 0.01 * u.squaredNorm();
  };
```

The terminal cost is optional:

```cpp
problem.terminal_cost =
  [](const Eigen::VectorXd& x) {
    return 10.0 * x.squaredNorm();
  };
```

For tracking problems, capture a reference sequence:

```cpp
problem.stage_cost =
  [reference](const Eigen::VectorXd& x,
              const Eigen::VectorXd& u,
              int step) {
    const Eigen::VectorXd e = x - reference[step];
    return e.squaredNorm() + 0.01 * u.squaredNorm();
  };
```

## Using From Another ROS2 Package

In `package.xml`:

```xml
<depend>mpc_controller</depend>
```

In `CMakeLists.txt`:

```cmake
find_package(mpc_controller REQUIRED)

target_link_libraries(your_target
  mpc_controller::mpc_controller
)
```

Then include:

```cpp
#include "mpc_controller/mpc.hpp"
```

## Solver Notes

This implementation uses coordinate search over the control sequence. It is simple, deterministic, dependency-light, and works with arbitrary C++ dynamics/cost callbacks.

It is not a full nonlinear programming solver. It does not guarantee a global optimum, and high-dimensional control problems can become expensive. For many experiments and small MPC problems, however, it is a useful baseline.

For the mathematical details, see [MPC_MATH.md](MPC_MATH.md).
