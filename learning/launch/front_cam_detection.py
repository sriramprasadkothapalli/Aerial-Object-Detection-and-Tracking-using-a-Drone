"""
Launches the YOLOv8 front camera node for object detection.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    yolo_node = Node(
        package='drone_control',        # <-- Replace this with your actual package name if different
        executable='front_cam_yolov8',   # Must match the name you installed or compiled
        name='yolov8_front_cam_node',
        output='screen',
        emulate_tty=True,                # For cleaner logs with colors if needed
        parameters=[]                    # You can add parameters here if your node takes any in future
    )

    return LaunchDescription([
        yolo_node
    ])
