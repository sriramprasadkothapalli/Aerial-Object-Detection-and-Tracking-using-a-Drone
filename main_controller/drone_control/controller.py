from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu, LaserScan
from gazebo_msgs.srv import SetEntityState
import rclpy
import math
import numpy as np
from functools import partial

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        self.get_logger().info("DroneControlNode initialized.")

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.get_logger().info("Velocity publisher set on /demo/cmd_vel.")

        # Subscribers
        self.create_subscription(Imu, '/demo/imu/out', self.imu_callback, 1)
        self.create_subscription(Pose, '/demo/gt_pose', self.pose_callback, 1)
        self.create_subscription(LaserScan, '/demo/laser_scanner/out', self.lidar_callback, 1)
        self.get_logger().info("Subscribed to IMU, GPS Pose, and LiDAR.")

        # Service client
        self.set_state_client = self.create_client(SetEntityState, "/demo/set_entity_state")

        # Internal state
        self.agent_position = np.array([1.0, 16.0], dtype=np.float32)
        self.agent_orientation = 0.0
        self.lidar_readings = np.full(180, 10.0, dtype=np.float32)

        self.robot_name = "Drone"
        self.set_state_done = False

    def send_velocity_command(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.vel_publisher.publish(msg)

    def imu_callback(self, msg: Imu):
        self.agent_orientation = 2 * math.atan2(msg.orientation.z, msg.orientation.w)

    def lidar_callback(self, msg: LaserScan):
        self.lidar_readings = np.array(msg.ranges, dtype=np.float32)
        self.lidar_readings[np.isinf(self.lidar_readings)] = 10.0

    def pose_callback(self, msg: Pose):
        x = np.clip(msg.position.x, -12, 12)
        y = np.clip(msg.position.y, -35, 21)
        self.agent_position = np.array([x, y], dtype=np.float32)

    def reset_robot_position(self, pose=[1.0, 16.0, -0.707, 0.707]):
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /demo/set_entity_state service...")

        request = SetEntityState.Request()
        request.state.name = self.robot_name
        request.state.pose.position.x = pose[0]
        request.state.pose.position.y = pose[1]
        request.state.pose.position.z = 1.0
        request.state.pose.orientation.z = pose[2]
        request.state.pose.orientation.w = pose[3]

        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0

        future = self.set_state_client.call_async(request)
        future.add_done_callback(partial(self._on_robot_state_set))

    def _on_robot_state_set(self, future):
        try:
            _ = future.result()
            self.set_state_done = True
        except Exception as e:
            self.get_logger().error(f"Failed to set robot state: {e}")

    def reset_target_position(self, position=[1.0, 10.0]):
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /demo/set_entity_state service...")

        request = SetEntityState.Request()
        request.state.name = "Target"
        request.state.pose.position.x = position[0]
        request.state.pose.position.y = position[1]
        request.state.pose.position.z = 1.0

        future = self.set_state_client.call_async(request)
        future.add_done_callback(partial(self._on_target_state_set))

    def _on_target_state_set(self, future):
        try:
            _ = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to set target state: {e}")
