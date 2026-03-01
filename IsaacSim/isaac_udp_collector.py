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

# 💥 核心修复区：删除了直接导入 Config 类的代码，改为官方推荐的动态解析器
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

# ==========================================
# 📡 UDP 非阻塞监听器 (充当虚拟手柄)
# ==========================================
class UDPTeleopReceiver:
    def __init__(self, port=8212, device="cuda:0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.device = device
        self.current_action = torch.zeros((1, 8), device=self.device) # 默认静止

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
            except Exception as e:
                pass
        
        # 自动衰减（如果没有持续收到指令，机械臂停止移动，防止暴走）
        self.current_action *= 0.9 
        return self.current_action

def main():
    print("[INFO] 正在从任务注册表动态加载配置...")
    env_cfg = parse_env_cfg("Isaac-Lift-Cube-Franka-IK-Rel-v0")
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = 3600.0

    print("DEBUG: 正在初始化环境 (这一步最容易卡住)...")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    print("DEBUG: 环境初始化成功！正在启动 UDP 接收器...")
    udp_receiver = UDPTeleopReceiver(device=env.device)
    
    print("===========================================")
    print("🚀 5090 数据采集站已就绪！监听端口: 8212")
    print("===========================================")
    
    # ==========================================
    # 💥 核心修复一：强行给环境挂载一个高清俯视摄像机
    # ==========================================
    from isaaclab.sensors import CameraCfg
    import isaaclab.sim as sim_utils
    env_cfg.scene.top_camera = CameraCfg(
        prim_path="/World/envs/env_.*/top_camera", # 挂载到每个环境的根节点
        update_period=0.0,
        height=480, width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        # 把摄像头放在桌子正上方 1米处，低头俯视 (模拟 ALOHA 视角)
        offset=CameraCfg.OffsetCfg(pos=(0.5, 0.0, 1.0), rot=(0.707, 0.0, 0.707, 0.0)) 
    )
    
    # env = ManagerBasedRLEnv(cfg=env_cfg)
    # udp_receiver = UDPTeleopReceiver(device=env.device)
    
    # print("===========================================")
    # print("🚀 5090 数据采集站已就绪！监听端口: 8212")
    # print("===========================================")
    
    obs, _ = env.reset()
    trajectory_images, trajectory_actions, trajectory_states = [], [], []
    demo_count = 0

    while simulation_app.is_running():
        # 1. 接收 Mac 指令
        raw_data = udp_receiver.get_action()
        
        # 💥 关键点：检查数据维度并提取命令
        # raw_data 的 shape 应该是 [1, 8]
        if raw_data.shape[1] >= 8:
            env_action = raw_data[:, :7]
            remote_cmd = raw_data[0, 7].item()
            
            # 只有当收到非 0 的有效命令时才打印并执行
            if abs(remote_cmd) > 0.5:
                if remote_cmd > 0:
                    print(f"🟢 [MANUAL SAVE] 收到保存请求，正在写入 HDF5...")
                    # 执行保存逻辑...
                else:
                    print(f"🔴 [MANUAL RESET] 收到重置请求，环境正在刷新...")
                    env.reset()
        else:
            env_action = raw_data[:, :7]
            remote_cmd = 0

        # 2. 仿真步进
        obs, rewards, dones, _, _ = env.step(env_action)
        
        # 3. 记录数据 (保持之前的存储逻辑)
        if "top_camera" in env.scene.sensors:
            img = env.scene.sensors["top_camera"].data.output["rgb"][0].cpu().numpy()
            trajectory_images.append(img)
        
        state = env.scene.articulations["robot"].data.joint_pos[0].cpu().numpy()
        trajectory_states.append(state)
        trajectory_actions.append(env_action[0].cpu().numpy())

        # 4. 💥 处理保存与重置逻辑
        # 我们用 remote_cmd 来强制触发
        should_save = (remote_cmd == 1)
        should_reset = (remote_cmd == -1)

        # 在 main 循环处理命令的地方：
        if should_reset:
            print("🗑️ 收到重置指令，清理环境...")
            env.reset()
            trajectory_images.clear()
            trajectory_actions.clear()
            trajectory_states.clear()
            # 💥 关键：执行完重置后，强行休眠 1 秒，并排空缓冲区里的过期垃圾指令
            time.sleep(1.0) 
            while True: # 排空网络缓冲区里残留的 -1.0
                try:
                    udp_receiver.sock.recvfrom(1024)
                except BlockingIOError:
                    break

        elif should_save:
            if len(trajectory_actions) < 10: # 防止误触保存空数据
                print("⚠️ 轨迹太短，跳过保存。")
            else:
                print(f"💾 指令：手动保存第 {demo_count+1} 条轨迹...")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"logs/demos/roarm_franka_{timestamp}.hdf5"
                os.makedirs("logs/demos", exist_ok=True)
                with h5py.File(filename, "w") as f:
                    f.create_dataset("obs/images/top", data=np.array(trajectory_images))
                    f.create_dataset("obs/state", data=np.array(trajectory_states))
                    f.create_dataset("actions", data=np.array(trajectory_actions))
                print(f"✅ 文件已写入: {filename}")
                demo_count += 1
            
            # 保存后必须重置
            env.reset()
            trajectory_images, trajectory_actions, trajectory_states = [], [], []
            time.sleep(0.5) # 给重置留一点时间

        # 官方默认重置判定 (比如积木掉了或抓太高了)
        elif dones[0]:
            print("🤖 环境自动判定完成，正在重置...")
            trajectory_images, trajectory_actions, trajectory_states = [], [], []

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()


"""
(base) lkk@rtx5090:/Developer/omniverselab$ cp IsaacSim/isaac_udp_collector.py ../IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p isaac_udp_collector.py --enable_cameras --livestream 2

"""