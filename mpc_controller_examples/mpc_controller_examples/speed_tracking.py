from scipy.optimize import minimize

# System Model is: v_i+1 = v_i + Ts * a_i
# v_0 = 0 m/s, a_-1 = 0 m/s^2 
# v_ref =  2 m/s, Ts = 1 s, N = 2
# Cost: J = (v_1 - v_ref)^2 + (v_2 - v_ref)^2 + a_0^2 + a_1^2
# Constrains: -1 <= a_i <= 1 m/s^2, 0 <= v_i <= 2 m/s, |Da_i| <= 0.5 m/s^2 
# U = [a_0, a_1], V = [v_1, v_2]
# Weights: q = r = 1 

# Initialize
v_init = 0.0
v_ref = 2.0
a_prev = 0.0
Ts = 1
N = 2

# Weights
Q = 1.0
R = 1.0

# Constrains:
v_min = 0.0
v_max = 2.0
a_min = -1.0
a_max = 1.0
da_max = 0.5


def predict(v_init: float, U: list, Ts: float, N: int):
    """
    @brief Model of the system. 
    @param v_init: First speed of the system.
    @param U: Input list.
    @param N: Horizon
    @return: Predicted speeds.
    """
    V = []
    v = v_init
    for a in U:
        v = v + Ts * a
        V.append(v)  

    return V          

def solve_mpc(v_init: float, a_prev: float):
    """
    @brief Calculates the optimal acceleration command for the current state.
    @param v_init: First speed (current speed) of the system.
    @param a_prev: Applied the previous acceleration command.
    """

    def cost(U: list):
        """
        @brief: Calculation of the cost function.
        @param U: Acceleration command list.
        @return Cost Function -> J   
        """
        V = predict(v_init, U, Ts, N)
        
        tracking_cost = 0.0
        input_cost = 0.0

        for i in range(0, N):
            tracking_cost += (V[i] - v_ref)**2
            input_cost += U[i]**2
        
        tracking_cost = Q * tracking_cost
        input_cost = R * input_cost

        return tracking_cost + input_cost

    def margins(U: list):
        V = predict(v_init, U, Ts, N)
        U_total = [a_prev, *U]

        values = []

        for i in range(0, N):
            da = U_total[i + 1] - U_total[i]
            
            # state_condition = state_condition and ((V[i] <= v_max) and (V[i] >= v_min))
            # input_condition = input_condition and ((U[i] <= a_max) and (U[i] >= a_min))
            # input_diff_condition = input_diff_condition and (abs(U_total[i + 1] - U_total[i]) <= da_max)
            values.extend([
                v_max - V[i],
                V[i] - v_min,
                a_max - U[i],
                U[i] - a_min,
                da_max - da,
                da_max + da,
            ])

        return values

    result = minimize(
        fun=cost,
        x0=[0.0] * N,
        method="SLSQP",
        constraints=[{"type": "ineq", "fun": margins}],
        options={"ftol": 1e-10, "maxiter": 200},
    )

    return result

print(solve_mpc(v_init, 0))

