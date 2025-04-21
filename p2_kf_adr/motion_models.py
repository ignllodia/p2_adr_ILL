import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # Define and return the 3x3 identity matrix A
        return np.eye(3)

    def control_input_matrix_B(mu, delta_t):
        # Define B using current theta and timestep delta_t
        theta = mu[2]
        B = np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]
        ])
        return B

    return state_transition_matrix_A, control_input_matrix_B

def velocity_motion_model_2():
    def A():
        # Define and return the 6x6 constant velocity model transition matrix
        return np.eye(6)

    def B(mu, dt):
        # Return 6x2 zero matrix (no control input used in pure KF)
        return np.zeros((6, 2))

    return A, B
