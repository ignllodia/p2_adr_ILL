import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from kalman_filter import KalmanFilter_2
import math

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        # Add noise configuration parameters
        proc_noise_std = [0.02, 0.02, 0.01, 0.02, 0.02, 0.01]  # Process noise
        obs_noise_std = [0.02, 0.02, 0.01, 0.02, 0.02, 0.01]  # Observation noise

        # Pass noise configurations to KalmanFilter_2
        self.kf = KalmanFilter_2(initial_state, initial_covariance, proc_noise_std, obs_noise_std)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

        # Initialize Visualizer
        self.visualizer = Visualizer()

    def odom_callback(self, msg):
        # Extract position, orientation, velocities from msg
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        orientation = math.atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
            1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2)
        )
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # Combine into a control input
        control_input = np.array([linear_velocity, angular_velocity])

        # Run predict() and update() of KalmanFilter_2
        self.kf.predict(control_input, dt=0.1)  # Assuming a fixed timestep

        measurement = np.array([
            position[0],
            position[1],
            orientation,
            linear_velocity,
            0.0,  # Assuming no lateral velocity
            angular_velocity
        ])
        self.kf.update(measurement)

        # Visualize the Kalman filter states
        real_pose = [position[0], position[1], orientation]
        estimated_pose = self.kf.mu[:3]  # Assuming the first 3 elements are x, y, theta
        covariance = self.kf.sigma[:3, :3]  # Assuming covariance for x, y, theta
        self.visualizer.update(real_pose, estimated_pose, covariance, step="update")

        # Publish estimated full state
        estimated_pose_msg = PoseWithCovarianceStamped()
        estimated_pose_msg.header.stamp = self.get_clock().now().to_msg()
        estimated_pose_msg.header.frame_id = "map"
        estimated_pose_msg.pose.pose.position.x = self.kf.mu[0]
        estimated_pose_msg.pose.pose.position.y = self.kf.mu[1]
        estimated_pose_msg.pose.pose.orientation.z = math.sin(self.kf.mu[2] / 2.0)
        estimated_pose_msg.pose.pose.orientation.w = math.cos(self.kf.mu[2] / 2.0)
        self.publisher.publish(estimated_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

