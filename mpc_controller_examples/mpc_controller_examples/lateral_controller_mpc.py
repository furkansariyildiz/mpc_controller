from scipy.optimize import minimize
import math

# System Model: Xnext = A * X + B * U
# X = [x, y, psi], U = [delta]

# Initialize
state_init = [0.0, 1.0, 0.0]
U = [math.radians(-5), math.radians(0)]
delta_prev = math.radians(0)
v = 4.0 # m/s
L = 2.0 # m
Ts = 0.1 # s
simulation_step = 100

# Weights
Qy = 10
Qpsi = 5 
Rdelta = 1 
Sdelta = 1 

# Constrains:
delta_max = math.radians(25) # rad
delta_rate_max = math.radians(60) # rad/s
delta_change_max = delta_rate_max * Ts # rad

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
    state = state_init
    x, y, psi = state
    for u in U:
        x = x + Ts * v * math.cos(psi)
        y = y + Ts * v * math.sin(psi)
        psi = psi + Ts * v / L * math.tan(u)
        X.append([x, y, psi])

    return X 

def solve_mpc(state_init: list, delta_prev: float):
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
        @return Cost Function -> J
        """
        X = predict(state_init, U, v, L, Ts)
        U_total = [delta_prev, *U]

        tracking_cost = 0.0
        input_cost = 0.0

        for i in range(0, len(U)):
            tracking_cost += (Qy * X[i][1]**2) + (Qpsi * X[i][2]**2) 
            input_cost += (Rdelta * U_total[i + 1]**2) + (Sdelta * (U_total[i + 1] - U_total[i])**2)

        return tracking_cost + input_cost

    def margins(U: list):
        """
        @brief: Calculation of margin function to use constrains on the optimizer.
        @param U: Road wheel angle command list.
        @return Margin list
        """
        X = predict(state_init, U, v, L, Ts)
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
        x0=[0.0] * len(U),
        method="SLSQP",
        constraints=[{"type": "ineq", "fun": margins}],
        options={"ftol": 1e-10, "maxiter": 200}
    )

    return result

if __name__ == '__main__':
    for i in range(0, simulation_step):
        result = solve_mpc(state_init, delta_prev)
        if result.success:
            print("Optimal road wheel angle command: ", result.x)
            print("Predicted states: ", predict(state_init, result.x, v, L, Ts))
            print("Minimum Cost:", result.fun)
            x_next = state_init[0] + Ts * v * math.cos(state_init[2])
            y_next = state_init[1] + Ts * v * math.sin(state_init[2])
            psi_next = state_init[2] + Ts * v / L * math.tan(result.x[0])
            
            state_init[0] = x_next
            state_init[1] = y_next
            state_init[2] = psi_next
            delta_prev = result.x[0]
        else:
            print("Could not find any solution.")