import argparse
import torch
import numpy as np
from PIL import Image

# Isaac Lab / Sim 核心库
from omni.isaac.lab.app import AppLauncher

# 1. 启动配置：必须在其他 omni 库导入前执行
parser = argparse.ArgumentParser(description="Isaac Lab Adapter for LeRobot Models")
parser.add_argument("--model_path", type=str, required=True, help="本地模型文件夹路径")
parser.add_argument("--policy_type", type=str, default="pi0", choices=["act", "pi0"])
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-Aloha-v0")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 导入 Isaac Lab 环境组件
import omni.isaac.lab_tasks  # 注册任务
from omni.isaac.lab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from omni.isaac.lab.utils.registration import make_env

# 导入 LeRobot 策略加载器
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.pi0.modeling_pi0 import PI0Policy

def main():
    # 2. 加载 LeRobot 策略
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"正在加载 {args_cli.policy_type} 模型: {args_cli.model_path}")
    
    if args_cli.policy_type == "act":
        policy = ACTPolicy.from_pretrained(args_cli.model_path)
    else:
        policy = PI0Policy.from_pretrained(args_cli.model_path)
    
    policy.to(device)
    policy.eval()

    # 3. 创建仿真环境
    # 我们使用 Isaac Lab 预设的 Aloha 任务环境
    env: ManagerBasedRLEnv = make_env(args_cli.task, device=device)
    obs, _ = env.reset()

    print("开始仿真评估... 按 Ctrl+C 退出")
    
    try:
        with torch.inference_mode():
            while simulation_app.is_running():
                # 4. 数据对齐：将 Isaac 的观测值转为 LeRobot 格式
                # 假设环境返回 'pixels' (相机画面) 和 'joint_pos' (本体状态)
                # 注意：实际格式取决于 env_cfg，这里展示标准转换逻辑
                
                # 处理图像 (假设是单个全局相机)
                # [Batch, H, W, C] -> [Batch, C, H, W] 并归一化
                img_tensor = obs["policy"]["pixels"].permute(0, 3, 1, 2).float() / 255.0
                
                # 构建模型输入字典
                lerobot_obs = {
                    "observation.image": img_tensor,
                    "observation.state": obs["policy"]["joint_pos"]
                }

                # 5. 模型推理
                # ACT 和 Pi0 通常输出一个动作块 (Action Chunk)
                action_chunk = policy.select_action(lerobot_obs)
                
                # 我们取动作块的第一步执行 (或者使用 Temporal Ensembling)
                action = action_chunk[:, 0, :]

                # 6. 执行动作并渲染
                obs, _, _, _, _ = env.step(action)
                
    except KeyboardInterrupt:
        print("仿真结束。")
    finally:
        env.close()
        simulation_app.close()

if __name__ == "__main__":
    main()