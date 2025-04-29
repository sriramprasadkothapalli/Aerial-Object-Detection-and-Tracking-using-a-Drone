import os
import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium.envs.registration import register
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold

class DroneTrainingNode(Node):
    def __init__(self):
        super().__init__(
            "drone_training_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

def main():
    # === Hyperparameters ===
    TOTAL_TIMESTEPS = 2_000_000
    REWARD_THRESHOLD = 500  # realistic threshold for simple reward
    VERBOSE = 1
    EVAL_FREQ = 10_000
    EVAL_EPISODES = 5

    PPO_PARAMS = {
        "n_steps": 2048,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "ent_coef": 0.01,
        "vf_coef": 0.5,
        "learning_rate": 3e-4,
        "clip_range": 0.2,
        "batch_size": 64,
        "n_epochs": 10,
        "tensorboard_log": "./drone_rl/logs",
        "verbose": VERBOSE
    }

    # === ROS Node Initialization ===
    rclpy.init()
    node = DroneTrainingNode()
    node.get_logger().info("Drone training node initialized.")

    # === Paths ===
    base_dir = os.getcwd()
    model_save_path = os.path.join(base_dir, "drone_rl", "model_rl")
    os.makedirs(model_save_path, exist_ok=True)

    # === Environment Registration ===
    register(
        id="DroneNavigationEnv-v0",
        entry_point="drone_rl.drone_env:DroneEnv",
        max_episode_steps=1000,
    )
    node.get_logger().info("Drone environment registered as DroneNavigationEnv-v0.")

    # === Environment Setup ===
    env = gym.make("DroneNavigationEnv-v0")
    env = Monitor(env)

    # === Callbacks Setup ===
    stop_training = StopTrainingOnRewardThreshold(REWARD_THRESHOLD, verbose=VERBOSE)
    eval_callback = EvalCallback(
        env,
        callback_on_new_best=stop_training,
        eval_freq=EVAL_FREQ,
        n_eval_episodes=EVAL_EPISODES,
        best_model_save_path=model_save_path,
        verbose=VERBOSE
    )

    # === PPO Model Setup ===
    model = PPO(
        "MultiInputPolicy",
        env,
        **PPO_PARAMS
    )

    # === Start Training ===
    node.get_logger().info("Starting PPO training...")

    try:
        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            reset_num_timesteps=True,
            callback=eval_callback,
            tb_log_name="PPO_Drone_Training"
        )
    except KeyboardInterrupt:
        node.get_logger().warn("Training interrupted! Saving current model...")
    finally:
        model.save(os.path.join(model_save_path, "PPO_Drone_Final"))
        node.get_logger().info("Model saved. Shutting down...")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
