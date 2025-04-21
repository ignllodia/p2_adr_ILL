import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]):
        self.mu = initial_state # Initial state estimate [x, y, theta]
        self.Sigma = initial_covariance # Initial uncertainty

        self.A, self.B = velocity_motion_model() # The action model to use. Returns A and B matrices

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        # Process noise covariance (R)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model() # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        # Observation noise covariance (Q)
        self.Q = np.diag(self.obs_noise_std ** 2)
            
    def predict(self, u, dt):
        # Predict the new mean (mu) using A, B, and control input u
        self.mu = self.A @ self.mu + self.B @ u

        # Predict the new covariance (Sigma) using A and R
        self.Sigma = self.A @ self.Sigma @ self.A.T + self.R

    def update(self, z):
        # Compute Kalman gain K
        S = self.C @ self.Sigma @ self.C.T + self.Q
        K = self.Sigma @ self.C.T @ np.linalg.inv(S)

        # Update the mean (mu) with the measurement z
        self.mu = self.mu + K @ (z - self.C @ self.mu)

        # Update the covariance (Sigma)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6):

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Initial uncertainty

        self.A, self.B = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        # Predict the new mean (mu) using A, B, and control input u (if provided)
        if u is not None:
            self.mu = self.A @ self.mu + self.B @ u
        else:
            self.mu = self.A @ self.mu

        # Predict the new covariance (Sigma) using A and R
        self.Sigma = self.A @ self.Sigma @ self.A.T + self.R

    def update(self, z):
        # Compute Kalman gain K
        S = self.C @ self.Sigma @ self.C.T + self.Q
        K = self.Sigma @ self.C.T @ np.linalg.inv(S)

        # Update the mean (mu) with the measurement z
        self.mu = self.mu + K @ (z - self.C @ self.mu)

        # Update the covariance (Sigma)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma
