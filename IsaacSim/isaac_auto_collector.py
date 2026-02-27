import argparse
import torch
import h5py
import numpy as np
import time
import os
from datetime import datetime

# 初始化 Isaac Lab
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description="Auto Expert Data Collection V3")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

# ==========================================
# 🔧 核心调参区：抓取点校准 (TCP Calibration)
# ==========================================
# panda_hand (手腕) 到 夹爪指尖 (Fingertips) 的距离补偿
OFFSET_Z = 0.115  # 往上抬 11.5cm，防止手腕撞桌子
OFFSET_X = 0.00   # 如果偏前/偏后，微调这里 (例如 0.02 或 -0.02)
OFFSET_Y = 0.00   # 如果偏左/偏右，微调这里

# 桌面的绝对安全高度 (基于手腕坐标系计算)
SAFE_Z_FLOOR = 0.12  # 任何时候手腕的 Z 轴不得低于此高度

def main():
    env_cfg = parse_env_cfg("Isaac-Lift-Cube-Franka-IK-Rel-v0")
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = 20.0 # 增加时间，允许它重试

    print("===========================================")
    print("🚀 V3 自我纠错专家系统 (带 TCP 校准)")
    print("===========================================")
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    robot = env.scene["robot"]
    ee_idx = robot.find_bodies("panda_hand")[0][0] 
    
    obs, _ = env.reset()
    trajectory_images, trajectory_actions, trajectory_states = [], [], []
    
    demo_count = 0
    max_demos = 50 
    
    state = "HOVER"
    state_timer = 0
    retry_count = 0 # 记录重试次数

    while simulation_app.is_running() and demo_count < max_demos:
        state_timer += 1
        
        # 获取积木和手腕的绝对坐标
        cube_pos = env.scene["object"].data.root_pos_w[0]
        ee_pos = robot.data.body_pos_w[0, ee_idx]
        
        action = torch.zeros((1, 7), device=env.device)
        grip_cmd = -1.0 # 默认张开 (-1.0 开, 1.0 闭)
        
        # 💥 加入抓取点补偿
        target_tcp = cube_pos.clone()
        target_tcp[0] += OFFSET_X
        target_tcp[1] += OFFSET_Y
        target_tcp[2] += OFFSET_Z 
        
        # 计算距离
        dist_xy = torch.norm(target_tcp[:2] - ee_pos[:2])
        dist_z = abs(ee_pos[2] - target_tcp[2])
        
        # --- 🛡️ 状态机与防撞机制 ---
        if state == "HOVER":
            target_pos = target_tcp.clone()
            target_pos[2] += 0.15 # 悬停在指尖目标上方 15cm
            grip_cmd = -1.0
            if dist_xy < 0.02 and dist_z < 0.16:
                state = "DESCEND"
                state_timer = 0
                
        elif state == "DESCEND":
            target_pos = target_tcp.clone()
            grip_cmd = -1.0
            
            # 🛡️ 地板防穿透钳制 (Clamping)
            target_pos[2] = max(target_pos[2], SAFE_Z_FLOOR)
            
            if dist_z < 0.02 or state_timer > 100:
                state = "GRASP"
                state_timer = 0
                
        elif state == "GRASP":
            target_pos = target_tcp.clone()
            target_pos[2] = max(target_pos[2], SAFE_Z_FLOOR)
            grip_cmd = 1.0 # 闭合夹爪
            
            if state_timer > 40: # 等待 40 帧抓稳
                state = "LIFT_AND_CHECK"
                state_timer = 0
                
        elif state == "LIFT_AND_CHECK":
            target_pos = target_tcp.clone()
            target_pos[2] += 0.25 # 提拔
            grip_cmd = 1.0
            
            # 🧠 自我纠错逻辑：提起来后检查积木的高度
            if state_timer > 60:
                cube_z = env.scene["object"].data.root_pos_w[0][2]
                if cube_z > 0.10: # 积木被提起来了！
                    print("🎉 确认抓取成功！正在入库...")
                    # 触发保存机制...
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"logs/demos/auto_demo_{demo_count:04d}_{timestamp}.hdf5"
                    os.makedirs("logs/demos", exist_ok=True)
                    with h5py.File(filename, "w") as f:
                        f.create_dataset("obs/state", data=np.array(trajectory_states))
                        f.create_dataset("actions", data=np.array(trajectory_actions))
                    print(f"💾 数据已入库: {filename} ({demo_count+1}/{max_demos})")
                    
                    demo_count += 1
                    env.reset()
                    trajectory_images, trajectory_actions, trajectory_states = [], [], []
                    state = "HOVER"
                    state_timer = 0
                    retry_count = 0
                else:
                    # 抓空了！
                    retry_count += 1
                    print(f"⚠️ 抓空了！积木还在桌面上。启动重试程序... (第 {retry_count} 次)")
                    if retry_count > 3:
                        print("❌ 连续重试失败，放弃当前回合。")
                        env.reset()
                        trajectory_images, trajectory_actions, trajectory_states = [], [], []
                        retry_count = 0
                    state = "HOVER" # 退回悬停状态重新瞄准
                    state_timer = 0
        
        # --- 🚀 柔性 P-Controller ---
        kp = 3.0 # 温和的增益
        xyz_action = (target_pos - ee_pos) * kp
        xyz_action = torch.clamp(xyz_action, -1.0, 1.0)
        
        action[0, 0] = xyz_action[0]
        action[0, 1] = xyz_action[1]
        action[0, 2] = xyz_action[2]
        # 强制姿态保持静止，不再扭麻花
        action[0, 3] = 0.0 
        action[0, 4] = 0.0
        action[0, 5] = 0.0
        action[0, 6] = grip_cmd      
        
        # 步进仿真
        obs, rewards, dones, _, _ = env.step(action)
        
        robot_state = robot.data.joint_pos[0].cpu().numpy()
        trajectory_states.append(robot_state)
        trajectory_actions.append(action[0].cpu().numpy())
        
        # 兜底超时重置
        if dones[0] and state != "HOVER":
            print("⏳ 触发环境官方超时，强制重置。")
            env.reset()
            trajectory_images, trajectory_actions, trajectory_states = [], [], []
            state = "HOVER"
            state_timer = 0
            retry_count = 0

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()

"""
(base) lkk@rtx5090:~$ conda activate isaac_lerobot
(isaac_lerobot) lkk@rtx5090:~$ cd /Developer/IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p isaac_auto_collector.py --enable_cameras --livestream 2
"""