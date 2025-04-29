"""
Launches Gazebo with a specified world and spawns a drone entity.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the package directory
    package_dir = get_package_share_directory('hospital_robot_spawner')

    # Set the Gazebo model path
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(package_dir, 'models')

    # Specify the world file
    world_path = '/home/sriram/drone_ws/src/Drone-RL-Control/sjtu_drone_description/worlds/garage.world'

    # Launch Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Spawn the drone into Gazebo
    spawn_drone_node = Node(
        package='drone_control',
        executable='spawn_drone',
        arguments=[
            'SJTU-DroneEnv',  # Model to load
            'drone_entity',   # Entity name
            '1',              # X position
            '16.0',           # Y position
            '0.0'             # Z position
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_drone_node
    ])
