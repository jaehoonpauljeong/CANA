import gym
import numpy as np
from stable_baselines3 import DQN
# Import your custom OMNeT++ socket environment here
from cana_env import CanaAppEnv

def evaluate_model(model_path, num_episodes=5):
    # 1. Initialize your custom OMNeT++ socket environment
    env = CanaAppEnv(port=4444) 
    
    # 2. Load the trained model
    print(f"Loading trained model from: {model_path}")
    model = DQN.load(model_path, env=env)
    
    # Metrics trackers for evaluation
    episode_rewards = []
    episode_packet_drops = []
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        done = False
        truncated = False
        total_reward = 0
        
        print(f"\n--- Starting Evaluation Episode {episode + 1} ---")
        
        while not (done or truncated):
            # Predict optimal action (deterministic=True)
            action, _states = model.predict(obs, deterministic=True)
            
            # Step the simulation forward
            obs, reward, done, truncated, info = env.step(action)
            total_reward += reward
            
        final_dropped = info.get("total_dropped", 0)
        
        episode_rewards.append(total_reward)
        episode_packet_drops.append(final_dropped)
        
        print(f"Episode {episode + 1} Finished! Reward: {total_reward}, Drops: {final_dropped}")
        
    env.close()

    # ========================================================================
    # NEW CODE: ADDED LINES TO GENERATE AND SAVE OVERALL EVALUATION SUMMARY
    # ========================================================================
    avg_reward = np.mean(episode_rewards)
    avg_drops = np.mean(episode_packet_drops)

    summary_text = f"""========================================================================
                      FANET RL MODEL EVALUATION SUMMARY
========================================================================
Environment Configuration:
- Simulation Environment: OMNeT++ / Simu5G
- Operating System: Ubuntu 22.04 LTS
- Interface Protocol: Non-blocking POSIX OS Sockets
- Agent Architecture: Deep Q-Network (DQN) - Stable-Baselines3

Evaluation Configurations:
- Total Run Episodes: {num_episodes}
- Execution Mode: Deterministic (Inference Phase)

========================================================================
                       KEY PERFORMANCE METRICS
========================================================================
- Average Cumulative Episode Reward: {avg_reward:.2f}
- Average Packet Drop Rate: {avg_drops:.2f}

Per-Episode Breakdown:
"""
    # Append individual data entries sequentially 
    for i in range(num_episodes):
        summary_text += f" -> Episode {i+1}: Reward = {episode_rewards[i]:.2f}, Packet Drops = {episode_packet_drops[i]}\n"

    summary_text += "========================================================================\n"

    # Save data structure to text/plain file named 'data.txt'
    output_filename = "data.txt"
    try:
        with open(output_filename, "w") as file:
            file.write(summary_text)
        print(f"\n[Success] Overall evaluation summary written directly to file: '{output_filename}'")
    except IOError as e:
        print(f"\n[Error] Failed to write evaluation file: {e}")
    # ========================================================================

if __name__ == "__main__":
    MODEL_PATH = "dqn_cana_drone_model" 
    evaluate_model(MODEL_PATH)