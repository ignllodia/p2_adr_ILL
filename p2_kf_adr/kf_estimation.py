import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.kf = KalmanFilter(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # Extract velocities and timestep
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z
        dt = 0.1  # Assuming a fixed timestep for simplicity

        # Run predict() and update() of KalmanFilter
        control_input = np.array([linear_velocity, angular_velocity])
        self.kf.predict(control_input, dt)

        # Simulate a measurement (for example, using odometry position)
        measurement = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            math.atan2(
                2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
                1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2)
            )
        ])
        self.kf.update(measurement)

        # Publish estimated state
        estimated_pose = PoseWithCovarianceStamped()
        estimated_pose.header.stamp = self.get_clock().now().to_msg()
        estimated_pose.header.frame_id = "map"
        estimated_pose.pose.pose.position.x = self.kf.mu[0]
        estimated_pose.pose.pose.position.y = self.kf.mu[1]
        estimated_pose.pose.pose.orientation.z = math.sin(self.kf.mu[2] / 2.0)
        estimated_pose.pose.pose.orientation.w = math.cos(self.kf.mu[2] / 2.0)
        self.publisher.publish(estimated_pose)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
