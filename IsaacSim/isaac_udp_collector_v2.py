import argparse
import socket
import torch
import h5py
import numpy as np
import time
import os
from datetime import datetime
import zmq

# Isaac Lab 核心组件
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description="UDP Teleop & Data Collection")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg, ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

# ==========================================
# 📡 UDP 连续动作接收器 (通道 A)
# ==========================================
class UDPTeleopReceiver:
    def __init__(self, port=8212, device="cuda:0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.device = device
        self.current_action = torch.zeros((1, 7), device=self.device)

    def get_action(self):
        latest_data = None
        try:
            while True:
                data, _ = self.sock.recvfrom(1024)
                latest_data = data
        except BlockingIOError:
            pass

        if latest_data:
            try:
                vals = [float(x) for x in latest_data.decode().split(',')]
                if len(vals) == 7: 
                    self.current_action = torch.tensor([vals], device=self.device)
            except Exception as e:
                print(f"⚠️ UDP 解析异常: {e}")
                
        return self.current_action

def main():
    print("[INFO] 正在加载环境配置...")
    env_cfg = parse_env_cfg("Isaac-Lift-Cube-Franka-IK-Rel-v0")
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = 3600.0

    print("DEBUG: 正在初始化环境...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    udp_receiver = UDPTeleopReceiver(device=env.device)
    
    print("===========================================")
    print("🚀 UDP 动作通道就绪 (端口 8212)")
    print("===========================================")
    
    obs, _ = env.reset()
    trajectory_images, trajectory_actions, trajectory_states = [], [], []
    demo_count = 0

    # 💥 初始化 ZMQ TCP 命令通道 (通道 B)
    zmq_ctx = zmq.Context()
    cmd_sock = zmq_ctx.socket(zmq.REP)
    cmd_sock.bind("tcp://0.0.0.0:8213")
    print("🛡️ TCP 命令通道就绪 (端口 8213)")

    while simulation_app.is_running():
        # --- 1. 获取摇杆动作 ---
        env_action = udp_receiver.get_action()
        
        # --- 2. 检查 TCP 命令 ---
        should_save = False
        should_reset = False
        try:
            cmd = cmd_sock.recv_string(flags=zmq.NOBLOCK)
            if cmd == "SAVE":
                print("\n🟢 [ZMQ] 收到绝对指令: 执行保存！")
                should_save = True
                cmd_sock.send_string("SAVE_SUCCESS")
            elif cmd == "RESET":
                print("\n🔴 [ZMQ] 收到绝对指令: 执行重置！")
                should_reset = True
                cmd_sock.send_string("RESET_SUCCESS")
        except zmq.error.Again:
            pass 

        # --- 3. 仿真步进 ---
        obs, rewards, dones, _, _ = env.step(env_action)
        
        # --- 4. 记录数据 ---
        # (确保你之前配置了相机，这里以默认数据结构为例)
        state = env.scene.articulations["robot"].data.joint_pos[0].cpu().numpy()
        trajectory_states.append(state)
        trajectory_actions.append(env_action[0].cpu().numpy())
        
        # 如果挂载了相机，则保存图像 (注：我移除了这部分的硬编码挂载，如果你需要图像，请确保环境配置里带有 top_camera)
        if hasattr(env.scene, "sensors") and "top_camera" in env.scene.sensors:
            img = env.scene.sensors["top_camera"].data.output["rgb"][0].cpu().numpy()
            trajectory_images.append(img)

        # --- 5. 处理重置或保存逻辑 ---
        if should_reset:
            print("🗑️ 正在清空环境...")
            env.reset()
            trajectory_images.clear()
            trajectory_actions.clear()
            trajectory_states.clear()
            time.sleep(0.2) 

        elif should_save:
            if len(trajectory_actions) < 10:
                print("⚠️ 轨迹太短，跳过保存。")
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"logs/demos/franka_{timestamp}.hdf5"
                os.makedirs("logs/demos", exist_ok=True)
                with h5py.File(filename, "w") as f:
                    f.create_dataset("obs/state", data=np.array(trajectory_states))
                    f.create_dataset("actions", data=np.array(trajectory_actions))
                    if trajectory_images:
                        f.create_dataset("obs/images/top", data=np.array(trajectory_images))
                print(f"✅ 文件已保存: {filename} (Demo {demo_count+1})")
                demo_count += 1
            
            env.reset()
            trajectory_images.clear()
            trajectory_actions.clear()
            trajectory_states.clear()
            time.sleep(0.2)

        elif dones[0]:
            print("🤖 环境自动判定完成，正在重置...")
            env.reset()
            trajectory_images.clear()
            trajectory_actions.clear()
            trajectory_states.clear()

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()

"""
(base) lkk@rtx5090:/Developer/omniverselab$ cp IsaacSim/isaac_udp_collector_v2.py ../IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p isaac_udp_collector_v2.py --enable_cameras --livestream 2

"""