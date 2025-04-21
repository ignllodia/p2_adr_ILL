from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2_kf_adr',
            executable='kf_estimation',
            name='kf_estimation',
            output='screen'
        ),
        Node(
            package='p2_kf_adr',
            executable='kf_estimation_vel',
            name='kf_estimation_vel',
            output='screen'
        )
    ])