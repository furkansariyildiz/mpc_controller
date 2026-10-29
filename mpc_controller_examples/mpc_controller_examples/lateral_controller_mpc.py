from scipy.optimize import minimize
from time import perf_counter
import math
import matplotlib.pylab as plt

# System Model: Xnext = A * X + B * U
# X = [x, y, psi], U = [delta]

# Initialize
state_init = [0.0, 1.0, 0.0]
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
    time_history = [0.0]
    state_history = [state_init.copy()]

    delta_history = []
    solve_time_history = []
    for i in range(0, simulation_step): # Simuatlion loop: Ts * simulation_step
        start = perf_counter()
        result = solve_mpc(state_init, delta_prev)
        solve_time = perf_counter() - start
        print(f"Solve time: {solve_time * 1000:.4f} ms")
        if result.success:
            # print("Optimal road wheel angle command: ", result.x)
            # print("Predicted states: ", predict(state_init, result.x, v, L, Ts))
            x_next = state_init[0] + Ts * v * math.cos(state_init[2])
            y_next = state_init[1] + Ts * v * math.sin(state_init[2])
            psi_next = state_init[2] + Ts * v / L * math.tan(result.x[0])

            print("Minimum Cost:", result.fun)
            print("Next state: ", [x_next, y_next, psi_next])

            print("Lateral Error: ", y_next)
            print("Heading Error: ", psi_next)
            print("\n--------------------------------\n")

            state_init[0] = x_next
            state_init[1] = y_next
            state_init[2] = psi_next

            time_history.append((i + 1) * Ts)
            state_history.append(state_init.copy())
            delta_history.append(float(result.x[0]))
            solve_time_history.append(solve_time)
            delta_prev = result.x[0]
        else:
            print("Could not find any solution.")
            break

    y_history = [state[1] for state in state_history]
    psi_deg_history = [math.degrees(state[2]) for state in state_history]
    delta_deg_history = [math.degrees(delta) for delta in delta_history]
    solve_ms_history = [solve_time * 1000 for solve_time in solve_time_history]

    command_times = time_history[:-1]

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(time_history, y_history)
    axes[0].axhline(0, color='black', linestyle='--')
    axes[0].set_ylabel("Lateral Error (m)")

    axes[1].plot(time_history, psi_deg_history)
    axes[1].axhline(0, color='black', linestyle='--')
    axes[1].set_ylabel("Heading Error (deg)")

    axes[2].step(command_times, delta_deg_history, where='post')
    axes[2].set_ylabel("Road Wheel Angle (deg)")

    axes[3].plot(command_times, solve_ms_history)
    axes[3].axhline(Ts * 1000, color='red', linestyle='--', label="Sampling Time")
    axes[3].set_ylabel("Solve Time (ms)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()


    y_samples = [state[1] for state in state_history[1:]]
    rms_y = math.sqrt(sum(y**2 for y in y_samples) / len(y_samples))
    crossing = max(0.0, -min(y_samples))

    mean_solve_ms = (1000 * sum(solve_time_history) / len(solve_time_history))
    max_solve_ms = 1000 * max(solve_time_history)
    commands = [0.0, *delta_history] 
    changes = [commands[i + 1] - commands[i] for i in range(len(commands) - 1)]

    print(f"N={N}")
    print(f"RMS Lateral Error: {rms_y:.4f} m")
    print(f"Maximum negative-side excursion: {crossing:.4f} m")
    print(f"Mean Solve Time: {mean_solve_ms:.4f} ms")
    print(f"Max Solve Time: {max_solve_ms:.4f} ms")
    print(f"Qy={Qy}, Qpsi={Qpsi}, Rdelta={Rdelta}, Sdelta={Sdelta}")
    print("Sum of squared changes:", sum(d**2 for d in changes))
    print("Max change [deg]:", math.degrees(max(abs(d) for d in changes)))

    for ax in axes:
        ax.grid(True)

    fig.suptitle(f"Lateral MPC - N={N}")
    fig.tight_layout()
    plt.show()