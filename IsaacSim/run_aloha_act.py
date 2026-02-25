# =========================================================================
# 💥 核心修复：绝对的导入优先级！
# 必须在没有任何 Isaac / Omniverse 代码之前，强行把系统级的 scipy 和 numpy 加载到内存
# =========================================================================
import numpy as np
import scipy
import trimesh
import torch

import argparse

# --- 1. Isaac Lab 核心启动组件 (现在它无法覆盖内存里的 numpy 了) ---
from isaaclab.app import AppLauncher

# 解析命令行参数 (如 --headless)
parser = argparse.ArgumentParser(description="Aloha ACT Policy in Isaac Lab")
parser.add_argument("--model_path", type=str, required=True, help="Path to local lerobot pretrained model folder")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动仿真 APP (RTX 5090 在此介入)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 2. 导入仿真所需的组件 (必须在 App 启动后) ---
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import Camera, CameraCfg

# 导入官方 Aloha 配置 
# 如果报错找不到 ALOHA_CFG，请告诉我，我们换成底层的 USD 路径
try:
    from isaaclab_assets.robots.aloha import ALOHA_CFG
except ImportError:
    print("[ERROR] 找不到 ALOHA_CFG，可能最新的 Isaac Lab 把它移除了。")
    # 预留降级方案...

from lerobot.policies.act.modeling_act import ACTPolicy

def main():
    """主运行循环"""
    device = args_cli.device
    
    # --- A. 加载训练好的大脑 (ACT Policy) ---
    print(f"[INFO] 正在加载 ACT 模型: {args_cli.model_path}")
    policy = ACTPolicy.from_pretrained(args_cli.model_path)
    policy.to(device)
    policy.eval()
    print("[INFO] ACT 模型加载完毕！")

    # --- B. 搭建仿真舞台 ---
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim_utils.design_scene(scene_cfg=sim_utils.SceneEntityCfg())

    # --- C. 召唤机器人 ---
    print("[INFO] 正在生成高保真 ALOHA 机器人...")
    aloha_cfg = ALOHA_CFG.copy()
    aloha_cfg.prim_path = "/World/Aloha"
    aloha_robot = Articulation(cfg=aloha_cfg)

    # --- D. 启动仿真 ---
    sim.reset()
    print("--- RTX 5090 光追渲染已开启 ---")

    for _ in range(10): sim.step()
    
    with torch.inference_mode():
        while simulation_app.is_running():
            # 简单步进，先不跑推理，看能不能渲染出来
            aloha_robot.write_data_to_sim()
            sim.step()
            aloha_robot.update(dt=sim_cfg.dt)

    simulation_app.close()

if __name__ == "__main__":
    main()
