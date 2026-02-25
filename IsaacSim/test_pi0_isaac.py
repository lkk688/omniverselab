import torch
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from omni.isaac.lab.app import AppLauncher

# 1. 启动仿真
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app

# 2. 导入环境组件 (必须在 AppLauncher 之后)
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.manager_based.manipulation.lift.config.aloha import AlohaCubeLiftEnvCfg
from omni.isaac.lab.utils.registration import make_env

def main():
    # 3. 加载服务器训好的 Pi0 大脑
    ckpt_path = "/Developer/aloha_pi0_last/last/pretrained_model" # 确认路径
    policy = PI0Policy.from_pretrained(ckpt_path).to("cuda")
    
    # 4. 创建 Isaac Lab 环境 (Aloha 抬升任务)
    env_cfg = AlohaCubeLiftEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    obs, _ = env.reset()

    print("--- RTX 5090 强力驱动：Pi0 脑机接口已启动 ---")

    with torch.inference_mode():
        while simulation_app.is_running():
            # 数据转换 (Isaac -> LeRobot)
            # 注意：这里需要根据实际的 env 观测空间做 reshape，Pi0 预期 (B, C, H, W)
            input_data = {
                "observation.image": obs["policy"]["pixels"].permute(0, 3, 1, 2).float() / 255.0,
                "observation.state": obs["policy"]["joint_pos"]
            }

            # 推理
            action = policy.select_action(input_data)[:, 0, :] # 取 Action Chunk 第一步

            # 执行
            obs, _, _, _, _ = env.step(action)

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()