# mpc_controller

This repository contains a generic native MPC solver for ROS2 (Humble) and Python reference examples for it.

| Package | Build type | Description |
|---|---|---|
| [mpc_controller](mpc_controller/) | `ament_cmake` | Generic C++ MPC solver library (Eigen). See its [README](mpc_controller/README.md) and [MPC_MATH.md](mpc_controller/MPC_MATH.md). |
| [mpc_controller_examples](mpc_controller_examples/) | `ament_python` | Python/SciPy reference examples used to prototype and cross-check MPC problems. See its [README](mpc_controller_examples/README.md). |

## Build

Clone the repository into a workspace `src/` folder. colcon finds both packages automatically.

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select mpc_controller mpc_controller_examples
source install/setup.bash
```
