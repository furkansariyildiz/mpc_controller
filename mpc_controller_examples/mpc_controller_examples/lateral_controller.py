import math
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# System Model: Xnext = A * X + B * U
# X = [x (m), y (m), psi (rad)], U = [delta (rad/s)]

# Initialize
delta_prev = math.radians(0)
v = 4.0 # m/s
L = 2.0 # m
Ts = 0.1 # s
simulation_step = 200
N = 10 # Horizon

# Weights
Qy = 10
Qpsi = 5 
Rdelta = 1 
Sdelta = 1 

# Constrains:
delta_max = math.radians(25) # rad
delta_rate_max = math.radians(60) # rad/s
delta_change_max = delta_rate_max * Ts # rad

def closest_point_on_segment(position: list, A: list, B: list):
    """
    @brief Find closest point on the segment.
    @param position: [x, y]
    @param A: [x, y]
    @param B: [x, y]
    @return [x_ref, y_ref, psi_ref]
    """
    d = [b - a for a, b in zip(A, B)]
    w = [p - a for a, p in zip (A, position)]

    t = sum(wi * di for wi, di in zip (w, d)) / sum(di * di for di in d)
    t = max(0, min(1, t))

    x_ref, y_ref = [a + t * di for a, di in zip(A, d)] 
    psi_ref = math.atan2(B[1] - A[1], B[0] - A[0])

    return x_ref, y_ref, psi_ref
    
def find_reference(position: list, path: list):
    """
    @brief Find reference with the path.
    @param position: [x, y]
    @param path: At least two point like [x, y]
    return [x_ref, y_ref, psi_ref]
    """
    min_distance_square = float(math.inf)
    x_ref, y_ref, psi_ref = [math.inf, math.inf, math.inf]
    x, y = position
    for i in range(0, len(path) - 1):
        x_ref_i, y_ref_i, psi_ref_i = closest_point_on_segment(position, path[i], path[i + 1])
        distance_square = (x - x_ref_i)**2 + (y - y_ref_i)**2
        if distance_square < min_distance_square:
            min_distance_square = distance_square
            x_ref, y_ref, psi_ref = x_ref_i, y_ref_i, psi_ref_i
    
    return x_ref, y_ref, psi_ref

def tracking_errors(state: list, reference: list):
    """
    @brief Calculates the tracking errors for the current state and reference.
    @param state: [x (m), y (m), psi (rad)] Current state of the system.
    @param reference: [x_ref (m), y_ref (m), psi_ref (rad)] Reference state of the system.
    @return Tracking errors [e_y (m), e_psi (rad)].
    """
    x, y, psi = state
    x_ref, y_ref, psi_ref = reference

    e_y = -math.sin(psi_ref) * (x - x_ref) + math.cos(psi_ref) * (y - y_ref)

    e_psi = math.atan2(math.sin(psi - psi_ref), math.cos(psi - psi_ref))

    return e_y, e_psi

# ------------------------ MPC Side ------------------------  
def predict(state_init: list, U: list, v: float, L: float, Ts: float):
    """
    @brief Model of the system.
    @param state_init: [x (m), y (m), psi (rad)] First state of the system.
    @param U: Input list. [delta] (rad/s) 
    @param v: Constant speed. (m/s)
    @param Ts: Sampling time. (s)
    @param N: Horizon
    @param L: Vehicle wheelbase. (m)
    @return: Predicted states.
    """
    X = []
    x, y, psi = state_init
    for u in U:
        x = x + Ts * v * math.cos(psi)
        y = y + Ts * v * math.sin(psi)
        psi = psi + Ts * v / L * math.tan(u)
        X.append([x, y, psi])

    return X 

def solve_mpc(state_init: list, delta_prev: float, path: list[list]):
    """
    @brief Calculates the optimal delta command for the current state.
    @param  state_init: [x (m), y (m), psi (rad)] First state of the system.
    @param delta_prev: Applied the previous delta command. (rad/s)
    @return: Optimal delta command.
    """

    def cost(U: list):
        """
        @brief Calculation of the cost function J
        @param U: Road wheel angle command list.
        @return Cost function -> J
        """
        X = predict(state_init, U, v, L, Ts)
        U_total = [delta_prev, *U]

        tracking_cost = 0.0
        input_cost = 0.0

        for i in range(0, len(U)):
            reference = find_reference(X[i][:2], path)
            e_y, e_psi = tracking_errors(X[i], reference)
            tracking_cost += Qy * e_y**2 + Qpsi * e_psi**2
            input_cost += (Rdelta * U_total[i + 1]**2) + (Sdelta * (U_total[i + 1] - U_total[i])**2)

        return tracking_cost + input_cost

    def margins(U: list):
        """
        @brief Calculation of margin function to use constrains on the optimiezr.
        @param U: Road wheel angle command list
        @return Margin list
        """
        U_total = [delta_prev, *U]

        values = []

        for i in range(0, len(U)):
            d_delta = U_total[i + 1] - U_total[i]

            values.extend([
                delta_max - U[i],
                U[i] + delta_max,
                delta_change_max - d_delta,
                d_delta + delta_change_max
            ])

        return values

    result = minimize(
        fun=cost,
        x0=[0.0] * N,
        method="SLSQP",
        constraints=[{"type": "ineq", "fun": margins}],
        options={"ftol": 1e-10, "maxiter": 200}
    )

    return result

if __name__ == '__main__':
    # y_ref = sin(0.2 * x), x: 0-100 m, sample of example: 0.5 m
    x_samples = [0.5 * i for i in range(0, 201)]
    path = [[x, math.sin(0.2 * x)] for x in x_samples]

    position = [4.0, 1.5]
    psi_init = math.atan2(0.2 * math.cos(0.2 * position[0]), 1.0)
    state = [*position, psi_init]
    state_history = [state.copy()]
    
    reference = find_reference(position, path)
    tracking_error = tracking_errors(state, reference)

    for i in range(0, simulation_step):
        result = solve_mpc(state, delta_prev, path)
        if result.success:
            x_next = state[0] + Ts * v * math.cos(state[2])
            y_next = state[1] + Ts * v * math.sin(state[2])
            psi_next = state[2] + Ts * v / L * math.tan(result.x[0])
            state = [x_next, y_next, psi_next]
            state_history.append(state.copy())
            delta_prev = result.x[0]

            reference = find_reference(state[:2], path)
            e_y, e_psi = tracking_errors(state, reference)

            print("Minimum Cost:", result.fun)
            print("Next state: ", state)
            print("Lateral Error: ", e_y)
            print("Heading Error: ", e_psi)
            print("Applied steering command [deg]: ", math.degrees(delta_prev))
            print("\n--------------------------------\n")
        else:
            print("MPC optimization failed at step ", i)
            break

    plt.figure(figsize=(12, 4))

    plt.plot(
        [point[0] for point in path],
        [point[1] for point in path],
        "k--",
        label="Reference path",
    )

    plt.plot(
        [s[0] for s in state_history],
        [s[1] for s in state_history],
        "b-",
        label="Vehicle path",
    )

    plt.scatter(
        state_history[0][0],
        state_history[0][1],
        color="green",
        label="Start",
        zorder=3,
    )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("MPC — Sinusoidal Path Tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
    
