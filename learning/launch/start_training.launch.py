"""
Launches the drone reinforcement learning node with specified training parameters.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the training parameters file
    config_dir = get_package_share_directory('drone_rl')
    training_params_file = os.path.join(config_dir, 'config', 'training_parameters.yaml')

    # Start the training node
    training_node = Node(
        package='drone_rl',
        executable='drone_rl',
        parameters=[training_params_file],
        output='screen'
    )

    return LaunchDescription([
        training_node
    ])
