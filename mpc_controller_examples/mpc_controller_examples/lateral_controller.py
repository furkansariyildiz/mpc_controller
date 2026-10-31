import math

def tracking_errors(state: list, reference: list):
    """
    @brief Calculates the tracking errors for the current state and reference.
    @param state: [x (m), y (m), psi (rad)] Current state of the system.
    @param reference: [x_ref (m), y_ref (m), psi_ref (rad)] Reference state of the system.
    @return Tracking errors [e_y (m), e_psi (rad)].
    """
    x, y, psi = state
    x_ref, y_ref, psi_ref = reference

    e_y = -math.sin(math.radians(psi_ref)) * (x - x_ref) + math.cos(math.radians(psi_ref)) * (y - y_ref)

    e_psi = math.atan2(math.sin(math.radians(psi - psi_ref)), math.cos(math.radians(psi - psi_ref)))

    return e_y, e_psi

if __name__ == '__main__':
    state = [4, 3, 30]
    reference = [4.4, 2.2, 26.565]

    print(tracking_errors(state, reference))
