
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='app',
            executable='follow_red_ball',
            name='follow_red_ball',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
