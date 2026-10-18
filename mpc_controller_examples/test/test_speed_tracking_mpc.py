from mpc_controller_examples import speed_tracking_mpc as mpc
import numpy as np
import pytest


def test_first_plan_matches_analytical_solution():
    # From rest, the rate limit caps the first command at DA_MAX = 0.5.
    # With a[0] fixed, minimizing (0.5 + a1 - 2)^2 + a1^2 gives a1 = 0.75.
    U, V, _ = mpc.solve_mpc(v_now=0.0, a_prev=0.0)

    np.testing.assert_allclose(U, [0.5, 0.75], atol=1e-5)
    np.testing.assert_allclose(V, [0.5, 1.25], atol=1e-5)


def test_closed_loop_stays_feasible_and_reaches_reference():
    v, a_prev = 0.0, 0.0

    for _ in range(30):
        U, _, _ = mpc.solve_mpc(v, a_prev)
        a = U[0]

        assert mpc.A_MIN - 1e-6 <= a <= mpc.A_MAX + 1e-6
        assert abs(a - a_prev) <= mpc.DA_MAX + 1e-6

        v += mpc.TS * a
        assert mpc.V_MIN - 1e-6 <= v <= mpc.V_MAX + 1e-6

        a_prev = a

    assert v == pytest.approx(mpc.V_REF, abs=1e-3)
