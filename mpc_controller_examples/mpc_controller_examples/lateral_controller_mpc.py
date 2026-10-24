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

# Weights
Qy = 10
Qpsi = 5 
Rdelta = 1 
Sdelta = 1 

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

            print("Lateral Error Cost: ", (Qy * X[i][1]**2))
            print("Heading Error Cost: ", (Qpsi * X[i][2]**2))
            print("Wheel Angle Command Cost: ", (Rdelta * U_total[i + 1]**2))
            print("Wheel Angle Command Rate Cost: ", (Sdelta * (U_total[i + 1] - U_total[i])**2))

        print("Total Cost: ", (tracking_cost + input_cost))
        return tracking_cost + input_cost

    return cost(U)
if __name__ == '__main__':
    print(solve_mpc(state_init, delta_prev))