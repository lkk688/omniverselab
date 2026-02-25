import argparse
import socket
import torch
import h5py
import numpy as np
import time
from datetime import datetime

# Isaac Lab 核心组件
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description="UDP Teleop & Data Collection")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg, ManagerBasedRLEnv
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import FrankaCubeLiftEnvCfg

# ==========================================
# 📡 UDP 非阻塞监听器 (充当虚拟手柄)
# ==========================================
class UDPTeleopReceiver:
    def __init__(self, port=8212, device="cuda:0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.device = device
        self.current_action = torch.zeros((1, 7), device=self.device) # 默认静止

    def get_action(self):
        try:
            # 清空缓冲区，只获取最新一帧 (保证零延迟)
            latest_data = None
            while True:
                data, _ = self.sock.recvfrom(1024)
                latest_data = data
        except BlockingIOError:
            pass

        if latest_data:
            try:
                action_list = list(map(float, latest_data.decode().split(',')))
                self.current_action = torch.tensor([action_list], device=self.device)
            except:
                pass
        
        # 自动衰减（如果没有持续收到指令，机械臂停止移动，防止暴走）
        self.current_action *= 0.9 
        return self.current_action

# ==========================================
# 🎬 主循环与数据采集 (HDF5 格式)
# ==========================================
def main():
    env_cfg = FrankaCubeLiftEnvCfg()
    env_cfg.scene.num_envs = 1
    # 强制使用相对坐标 IK (适配摇杆和主从偏移)
    env_cfg.actions.arm_action = env_cfg.actions.arm_action.__class__(
        command_type="pose_rel", use_jacobian=True
    )
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    udp_receiver = UDPTeleopReceiver(device=env.device)
    
    print("===========================================")
    print("🚀 5090 UDP 数据采集基站就绪！监听端口: 8212")
    print("===========================================")
    
    obs, _ = env.reset()
    
    # 建立本地缓冲，用于保存这一个 Demo 的轨迹
    trajectory_images = []
    trajectory_actions = []
    trajectory_states = []
    
    demo_count = 0
    max_demos = 50 # 想录几条改这里

    while simulation_app.is_running():
        # 1. 接收 Mac 端指令
        action = udp_receiver.get_action()
        
        # 2. 仿真步进
        obs, rewards, dones, _, _ = env.step(action)
        
        # 3. 收集当前帧数据 (适配 LeRobot/Robomimic 格式)
        # 获取摄像机画面 (这里假设你开启了 --enable_cameras)
        if "policy" in obs and "rgb" in obs["policy"]:
             img = obs["policy"]["rgb"][0].cpu().numpy()
             trajectory_images.append(img)
             
        # 获取当前机器人的真实关节状态 (9个自由度)
        state = env.scene.articulations["robot"].data.joint_pos[0].cpu().numpy()
        trajectory_states.append(state)
        trajectory_actions.append(action[0].cpu().numpy())

        # 4. 检查是否抓取成功 (Env 重置)
        if dones[0]:
            print(f"🎉 成功完成一次抓取! (Demo {demo_count+1}/{max_demos})")
            
            # --- 保存为 HDF5 ---
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"logs/demos/roarm_franka_{timestamp}.hdf5"
            with h5py.File(filename, "w") as f:
                f.create_dataset("obs/images/top", data=np.array(trajectory_images))
                f.create_dataset("obs/state", data=np.array(trajectory_states))
                f.create_dataset("actions", data=np.array(trajectory_actions))
            print(f"💾 数据已保存至: {filename}")
            
            # 清空缓存，准备下一次录制
            trajectory_images.clear()
            trajectory_actions.clear()
            trajectory_states.clear()
            demo_count += 1
            
            if demo_count >= max_demos:
                break

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()

"""
./isaaclab.sh -p isaac_udp_collector.py --enable_cameras --livestream 1
"""