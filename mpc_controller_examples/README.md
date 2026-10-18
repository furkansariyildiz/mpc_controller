# mpc_controller_examples

Python reference examples for the [mpc_controller](../mpc_controller/) library.

The examples solve small MPC problems with `scipy.optimize.minimize` (SLSQP). They are used to prototype problem formulations and to produce reference results that the C++ coordinate-search solver can be compared against.

## Examples

| Executable | Description |
|---|---|
| `speed_tracking_mpc` | 1D speed tracking: state is speed, control is acceleration. Horizon `N = 2`, with acceleration bounds, speed bounds, and an acceleration rate limit (`DA_MAX`). Runs a closed-loop simulation and prints each step. |

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mpc_controller_examples
source install/setup.bash
```

## Run

```bash
ros2 run mpc_controller_examples speed_tracking_mpc
```

## Test

```bash
colcon test --packages-select mpc_controller_examples
colcon test-result --verbose
```

`test/test_speed_tracking_mpc.py` checks the first plan against the analytical solution and verifies that the closed loop respects all constraints and converges to `V_REF`.

## Comparing With The C++ Solver

`mpc_controller` only supports bounds on the control vector (see [MPC_MATH.md](../mpc_controller/MPC_MATH.md), section 10). Speed bounds and the rate limit used here have no direct equivalent. To compare results, either remove those constraints from the Python problem or add them as penalty terms in the C++ stage cost.
