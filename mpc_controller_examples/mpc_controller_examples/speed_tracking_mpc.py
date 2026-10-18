import numpy as np
from scipy.optimize import minimize

# Model and prediction horizon
TS = 1.0
N = 2

# Reference and cost weights
V_REF = 2.0
Q = 1.0
R = 1.0

# Constraints
A_MIN, A_MAX = -1.0, 1.0
V_MIN, V_MAX = 0.0, 2.0
DA_MAX = 0.5


def predict(v_now, U):
    """Compute future speeds from the current speed and acceleration plan."""
    speeds = []
    v = v_now

    for a in U:
        v = v + TS * a
        speeds.append(v)

    return np.array(speeds)


def solve_mpc(v_now, a_prev):
    """Compute the optimal acceleration plan for the current state."""

    def cost(U):
        V = predict(v_now, U)

        tracking_cost = Q * np.sum((V - V_REF)**2)
        input_cost = R * np.sum(U**2)

        return tracking_cost + input_cost

    def margins(U):
        V = predict(v_now, U)

        # The first difference is taken relative to the previously applied command.
        dU = np.diff(np.r_[a_prev, U])

        # All elements must be >= 0.
        return np.concatenate((
            V - V_MIN,
            V_MAX - V,
            DA_MAX - dU,
            DA_MAX + dU,
        ))

    result = minimize(
        cost,
        x0=np.zeros(N),
        method="SLSQP",
        bounds=[(A_MIN, A_MAX)] * N,
        constraints=[{"type": "ineq", "fun": margins}],
        options={"ftol": 1e-10, "maxiter": 200},
    )

    if not result.success:
        raise RuntimeError(result.message)

    # Check the constraints again with a numerical tolerance.
    if (
        np.min(margins(result.x)) < -1e-6
        or np.min(result.x - A_MIN) < -1e-6
        or np.min(A_MAX - result.x) < -1e-6
    ):
        raise RuntimeError("Constraint violation: solution not applied.")

    return result.x, predict(v_now, result.x), result.fun


def main():
    # Closed-loop simulation
    v = 0.0
    a_prev = 0.0

    print(" k | v_now | a_apply | v_next")

    for k in range(8):
        U, V, J = solve_mpc(v, a_prev)

        if k == 0:
            print("Initial plan:", np.round(U, 6))
            print("Initial prediction:", np.round(V, 6))
            print("Initial cost:", round(J, 6))

        # Apply only the first command of the plan.
        a_apply = U[0]

        # Advance the ideal system by one sample time.
        v_next = v + TS * a_apply

        print(
            f"{k:2d} | {v:5.3f} | "
            f"{a_apply:7.3f} | {v_next:6.3f}"
        )

        # Initial conditions for the next optimization
        v = v_next
        a_prev = a_apply


if __name__ == "__main__":
    main()
