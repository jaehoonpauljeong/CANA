import gymnasium as gym
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor

# Import your custom environment class from cana_env.py
from cana_env import CanaAppEnv

def main():
    print("[Agent] Initializing DQN Core model architecture...")
    
    # 1. Instantiate the custom environment
    raw_env = CanaAppEnv(host="127.0.0.1", port=4444)
    
    # 2. Wrap the environment with Monitor to track rewards and episode data
    monitored_env = Monitor(raw_env)
    
    # 3. Wrap in a DummyVecEnv as expected by Stable-Baselines3
    env = DummyVecEnv([lambda: monitored_env])
    
    # 4. Initialize the DQN Agent with Live TensorBoard logging added
    LOG_DIR = "./dqn_cana_tensorboard/"
    
    model = DQN(
        "MlpPolicy", 
        env, 
        verbose=1,
        learning_rate=1e-3,
        buffer_size=10000,
        learning_starts=100,
        batch_size=32,
        device="cpu",
        tensorboard_log=LOG_DIR  # <-- Added for real-time tracking
    )
    
    print("[Agent] Beginning training iterations...")
    try:
        # This will trigger env.reset(), forcing Python to wait for OMNeT++ to connect
        model.learn(total_timesteps=50000, log_interval=10)
    except KeyboardInterrupt:
        print("\n[Agent] Training interrupted by user.")
    finally:
        print("[Agent] Saving model checkpoint...")
        model.save("dqn_cana_drone_model")
        env.close()
        print("[Agent] Cleanup complete. Sockets released successfully.")

if __name__ == "__main__":
    main()