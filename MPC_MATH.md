# Mathematical Formulation

This document describes the generic MPC solver implemented in this package. The solver is intentionally model-agnostic: all system-specific mathematics live in the user-provided dynamics and cost callbacks.

## 1. Discrete-Time System

The solver assumes a discrete-time prediction model:

```text
x_{k+1} = f(x_k, u_k, dt, k)
```

where:

```text
x_k in R^n
u_k in R^m
k = 0, 1, ..., N - 1
```

`n` is the state dimension and `m` is the control dimension. The solver does not need to know the physical meaning of either vector.

The function `f` is supplied by the user:

```cpp
Vector dynamics(const Vector& x, const Vector& u, double dt, int k);
```

## 2. Prediction And Control Horizons

Prediction horizon:

```text
N
```

Control horizon:

```text
N_c, with 1 <= N_c <= N
```

The optimization variables are only the first `N_c` control vectors:

```text
U_dec =
[
  u_0,
  u_1,
  ...,
  u_{N_c-1}
]
```

For prediction steps beyond the control horizon, the last decision control is held constant:

```text
u_k = u_{N_c-1}, for k >= N_c
```

Equivalently:

```text
u_k = U_dec[min(k, N_c - 1)]
```

## 3. Optimization Problem

The generic MPC problem solved by this package is:

```text
minimize    J(U_dec)
subject to  x_0 = x_initial
            x_{k+1} = f(x_k, u_k, dt, k)
            u_min <= u_i <= u_max, for i = 0, ..., N_c - 1
```

The objective is:

```text
J(U_dec) =
  sum_{k=0}^{N-1} l(x_k, u_k, k)
  + l_f(x_N)
```

where:

```text
l(x_k, u_k, k)  = user-provided stage cost
l_f(x_N)        = optional user-provided terminal cost
```

If no terminal cost is provided:

```text
l_f(x_N) = 0
```

## 4. Control Bounds

For each control component:

```text
u_min_j <= u_{i,j} <= u_max_j
```

Vector form:

```text
u_min <= u_i <= u_max
```

The solver enforces this by clamping each candidate control:

```text
u_i <- min(max(u_i, u_min), u_max)
```

If a lower or upper bound vector is omitted, that side is treated as unbounded.

## 5. Forward Rollout

Given an initial state and a decision control sequence:

```text
x_0 = x_initial
```

the solver expands the control sequence:

```text
u_k = U_dec[min(k, N_c - 1)]
```

then rolls the dynamics forward:

```text
x_1 = f(x_0, u_0, dt, 0)
x_2 = f(x_1, u_1, dt, 1)
...
x_N = f(x_{N-1}, u_{N-1}, dt, N-1)
```

The predicted trajectory returned in `Result::states` is:

```text
X =
[
  x_0,
  x_1,
  ...,
  x_N
]
```

The expanded controls returned in `Result::controls` are:

```text
U =
[
  u_0,
  u_1,
  ...,
  u_{N-1}
]
```

## 6. Cost Evaluation

During rollout, the solver accumulates:

```text
J = 0
```

For each prediction step:

```text
J <- J + l(x_k, u_k, k)
```

After the final state:

```text
J <- J + l_f(x_N)
```

This final value is returned as:

```text
Result::cost
```

## 7. Coordinate-Search Optimization

The solver does not use symbolic gradients. It improves the control sequence by coordinate search.

Let:

```text
U_dec in R^{N_c x m}
```

The search begins from one of:

```text
U_dec = problem.initial_controls
```

or, if available:

```text
U_dec = previous shifted solution
```

or otherwise:

```text
U_dec = [u_nominal, u_nominal, ..., u_nominal]
```

At each iteration, with search step `alpha`, the solver visits every scalar decision variable:

```text
u_{i,j}
```

and tests two candidates:

```text
u_{i,j}^{+} = u_{i,j} + alpha
u_{i,j}^{-} = u_{i,j} - alpha
```

Each candidate is clamped to bounds:

```text
u_{i}^{candidate} <- clamp(u_i^{candidate}, u_min, u_max)
```

Then the full trajectory cost is recomputed:

```text
J^{candidate} = rollout_cost(U_dec^{candidate})
```

If:

```text
J^{candidate} < J^{best}
```

the candidate is accepted:

```text
U_dec <- U_dec^{candidate}
J^{best} <- J^{candidate}
```

After all scalar variables are visited, the search step is reduced:

```text
alpha <- alpha * step_decay
```

If no variable improved the cost, it is reduced more aggressively:

```text
alpha <- alpha * step_decay * 0.5
```

The loop stops when either:

```text
iteration_count >= max_iterations
```

or:

```text
alpha < min_step_size
```

## 8. Warm Start

If warm start is enabled, the optimized decision sequence is stored after every solve.

Suppose the previous optimized sequence is:

```text
[
  u_0^*,
  u_1^*,
  ...,
  u_{N_c-2}^*,
  u_{N_c-1}^*
]
```

Before the next solve, it is shifted:

```text
[
  u_1^*,
  u_2^*,
  ...,
  u_{N_c-1}^*,
  u_{N_c-1}^*
]
```

This is a standard MPC trick: the previous future plan is often a good initial guess for the next time step.

## 9. Double-Integrator Demo

The demo uses:

```text
x_k =
[
  p_k
  v_k
]
```

where:

```text
p_k = position
v_k = velocity
```

The control is:

```text
u_k =
[
  a_k
]
```

where:

```text
a_k = acceleration
```

The continuous-time system is:

```text
d/dt [p] = [v]
d/dt [v] = [a]
```

Using a constant-acceleration discretization:

```text
p_{k+1} = p_k + dt * v_k + 0.5 * dt^2 * a_k
v_{k+1} = v_k + dt * a_k
```

Matrix form:

```text
x_{k+1} = A x_k + B u_k
```

with:

```text
A =
[
  1  dt
  0  1
]

B =
[
  0.5 * dt^2
  dt
]
```

The target is:

```text
x_ref =
[
  3
  0
]
```

The demo stage cost is:

```text
l(x_k, u_k) =
  (x_k - x_ref)^T Q (x_k - x_ref)
  + u_k^T R u_k
```

with:

```text
Q =
[
  8  0
  0  1
]

R =
[
  0.05
]
```

The terminal cost is:

```text
l_f(x_N) =
  (x_N - x_ref)^T Q_f (x_N - x_ref)
```

with:

```text
Q_f =
[
  25  0
  0   5
]
```

The acceleration bound is:

```text
-2 <= a_k <= 2
```

At each receding-horizon step:

```text
1. solve MPC from current x
2. apply first acceleration a_0
3. update x using x_{k+1} = A x_k + B a_0
4. repeat
```

## 10. Important Limitations

This solver is intentionally simple.

It does not solve a quadratic program or nonlinear program with KKT conditions. It does not compute gradients, Hessians, or constraint Jacobians. It only tests nearby control candidates and keeps the ones that reduce the user-defined rollout cost.

That makes it easy to use with arbitrary C++ models, but it also means:

- no global optimum guarantee,
- no convexity exploitation,
- no explicit state constraint handling,
- higher cost for high-dimensional control vectors,
- tuning depends on `initial_step_size`, `step_decay`, and `max_iterations`.
