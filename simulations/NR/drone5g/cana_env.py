import gymnasium as gym
from gymnasium import spaces
import socket
import numpy as np

class CanaAppEnv(gym.Env):
    def __init__(self, host="127.0.0.1", port=4444):
        super(CanaAppEnv, self).__init__()
        
        # Actions: 0 = Fast (0.2s), 1 = Balanced (1.0s), 2 = Conservative (5.0s)
        self.action_space = spaces.Discrete(3)
        self.action_map = {0: "0.2", 1: "1.0", 2: "5.0"}
        
        # Observations: [Packets Sent, Packets Received, Packets Dropped]
        self.observation_space = spaces.Box(low=0, high=100000, shape=(3,), dtype=np.int32)
        
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((host, port))
        self.server_sock.listen(1)
        self.client_sock = None
        
        # Fix Bug 1: Initialize self.state at boot time
        self.state = np.array([0, 0, 0], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.client_sock:
            try:
                self.client_sock.close()
            except Exception:
                pass
            self.client_sock = None
            
        print(f"[Gym Server] Awaiting connection on port {self.server_sock.getsockname()[1]}...")
        
        self.client_sock, addr = self.server_sock.accept()
        print(f"[Gym Server] Simulation connected successfully from {addr}")
        
        # Fix Bug 2: Explicitly declare non-blocking mode so exceptions can trigger
        self.client_sock.setblocking(False)
        
        # Give a small pause to pull initialization handshake safely
        try:
            raw_data = self.client_sock.recv(1024).decode().strip()
            obs = self._parse_data(raw_data)
        except Exception:
            obs = np.array([0, 0, 0], dtype=np.int32)
            
        # Ensure state tracker stays perfectly matched 
        self.state = obs.astype(np.float32)
        return obs, {}

    def step(self, action):
        action_string = f"{action}\n"
        
        # 1. Action dispatch phase
        try:
            self.client_sock.sendall(action_string.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError):
            print("\n[Gym Server] OMNeT++ closed the connection or exited prematurely.")
            return self.state, 0.0, False, True, {"total_dropped": int(self.state[2])}
    
        # 2. Resilient waiting telemetry capture loop
        data = b""
        while True:
            try:
                data = self.client_sock.recv(1024)
                if not data:
                    print("[Gym Server] Socket closed by simulation termination.")
                    return self.state, 0.0, True, True, {"total_dropped": int(self.state[2])}
                break
            except (BlockingIOError, InterruptedError):
                continue
            except Exception as e:
                print(f"[Gym Server] Unexpected socket error during recv: {e}")
                return self.state, 0.0, True, True, {"total_dropped": int(self.state[2])}

        # 3. Telemetry tracking evaluation
        try:
            metrics_line = data.decode('utf-8').strip()
            if "\n" in metrics_line:
                metrics_line = metrics_line.split("\n")[-1]
              
            sent, received, dropped = map(int, metrics_line.split(','))
        
            # Safely save changes down to local state trackers
            self.state = np.array([sent, received, dropped], dtype=np.float32)
            
            # Compute custom evaluation reward mapping
            reward = float(received - (dropped * 2)) 
            
            terminated = bool(dropped >= 200)
            truncated = False
            info = {"total_dropped": dropped}
        
            return self.state, reward, terminated, truncated, info

        except ValueError:
            return self.state, 0.0, False, False, {"total_dropped": int(self.state[2])}

    def _parse_data(self, raw_str):
        try:
            cleaned = "".join([c for c in raw_str if c.isdigit() or c == ","])
            parts = [int(x) for x in cleaned.split(",") if x.strip()]
            if len(parts) == 3:
                return np.array(parts, dtype=np.int32)
        except Exception:
            return np.array([0, 0, 0], dtype=np.int32)
            
        return np.array([0, 0, 0], dtype=np.int32)

    def close(self):
        if self.client_sock:
            try: self.client_sock.close()
            except Exception: pass
        try: self.server_sock.close()
        except Exception: pass