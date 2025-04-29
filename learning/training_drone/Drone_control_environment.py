import math
import numpy as np
import rclpy
from gymnasium import Env
from gymnasium.spaces import Dict, Box
from drone_control.drone_control import DroneController


class StaticDroneEnv(DroneController, Env):
    """
    A minimal Gymnasium-compliant ROS 2 drone RL environment using fixed start and target positions,
    basic observation space, and a simple reward structure.
    """

    def __init__(self):
        super().__init__()

        self.get_logger().info("Initialized StaticDroneEnv node")

        # Fixed environment setup (Randomization = 0)
        self.agent_start_pose = [1.0, 10.0, -0.506, 0.405]
        self.target_position = np.array([5.0, 6.0], dtype=np.float32)

        # Reward and success/failure thresholds
        self.min_target_distance = 0.22
        self.min_obstacle_distance = 0.45

        # Velocity settings
        self.max_linear_vel = 1.0
        self.min_linear_vel = 0.0
        self.angular_vel = 1.0

        # Normalization flags
        self.normalize_obs = True
        self.normalize_act = True

        # Step counters
        self.step_count = 0
        self.episode_count = 0

        # Action space
        self.action_space = Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)

        # Observation space
        if self.normalize_obs:
            self.observation_space = Dict({
                "agent": Box(low=np.array([0, 0]), high=np.array([6, 1]), dtype=np.float32),
                "laser": Box(low=0, high=1, shape=(180,), dtype=np.float32),
            })
        else:
            self.observation_space = Dict({
                "agent": Box(low=np.array([0, -math.pi]), high=np.array([60, math.pi]), dtype=np.float32),
                "laser": Box(low=0, high=np.inf, shape=(180,), dtype=np.float32),
            })

    def step(self, action):
        self.step_count += 1

        # Convert normalized action to physical values
        action = self._denormalize_action(action)
        self.send_velocity_command(action)

        # Wait for updated data
        self._spin_until_data_ready()
        self._update_relative_target_position()

        obs = self._get_observation()
        info = self._get_info()
        reward = self._calculate_reward(info)

        done = (info["distance"] < self.min_target_distance) or \
               (np.any(info["laser"] < self.min_obstacle_distance))

        return obs, reward, done, False, info

    def reset(self, seed=None, options=None):
        self.episode_count += 1
        self.step_count = 0

        self.reset_agent_pose()
        self._spin_until_data_ready()
        self._update_relative_target_position()

        obs = self._get_observation()
        info = self._get_info()

        return obs, info

    def close(self):
        self.get_logger().info(f"Episodes run: {self.episode_count}")
        self.destroy_node()

    # === Utility Methods ===

    def reset_agent_pose(self):
        self._done_set_rob_state = False
        self.call_set_robot_state_service(self.agent_start_pose)
        while not self._done_set_rob_state:
            rclpy.spin_once(self)

    def _spin_until_data_ready(self):
        self._done_location = False
        self._done_laser = False
        self._done_orientation = False
        while not (self._done_location and self._done_laser and self._done_orientation):
            rclpy.spin_once(self)

    def _update_relative_target_position(self):
        dx = self.target_position[0] - self._agent_location[0]
        dy = self.target_position[1] - self._agent_location[1]
        theta = self._agent_orientation

        rel_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        rel_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        distance = math.sqrt(rel_x**2 + rel_y**2)
        angle = math.atan2(rel_y, rel_x)

        self._polar_coords = np.array([distance, angle], dtype=np.float32)
        self._relative_angle = angle

    def _get_observation(self):
        obs = {
            "agent": self._polar_coords,
            "laser": self._laser_reads
        }
        return self._normalize_observation(obs) if self.normalize_obs else obs

    def _get_info(self):
        return {
            "distance": np.linalg.norm(self._agent_location - self.target_position),
            "laser": self._laser_reads,
            "angle": self._relative_angle
        }

    def _calculate_reward(self, info):
        if info["distance"] < self.min_target_distance:
            self.get_logger().info("🎯 Reached target")
            return 1.0
        elif np.any(info["laser"] < self.min_obstacle_distance):
            self.get_logger().info("💥 Hit obstacle")
            return -1.0
        return 0.0

    def _normalize_observation(self, obs):
        obs["agent"][0] /= 10.0
        obs["agent"][1] = (obs["agent"][1] + math.pi) / (2 * math.pi)
        obs["laser"] = obs["laser"] / 10.0
        return obs

    def _denormalize_action(self, norm_action):
        linear = ((self.max_linear_vel * (norm_action[0] + 1)) +
                  (self.min_linear_vel * (1 - norm_action[0]))) / 2
        angular = self.angular_vel * norm_action[1]
        return np.array([linear, angular], dtype=np.float32)
