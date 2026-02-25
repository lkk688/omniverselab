import argparse
# ====================================================================
# 1. 绝对标准的启动顺序：必须首先初始化 AppLauncher，不能有任何其他导入
# ====================================================================
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="ACT Policy controlling Franka")
parser.add_argument("--model_path", type=str, required=True, help="ACT 模型路径")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动引擎
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ====================================================================
# 2. 引擎启动后，再导入相关的组件 (此时 Omniverse 的底层环境已经安全建立)
# ====================================================================
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import Camera, CameraCfg
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG

# 直接导入模型结构，而不是使用基于 draccus 的预训练加载器
from lerobot.policies.act.modeling_act import ACTPolicy

def main():
    device = args_cli.device

    # --- A. 搭建 5090 仿真环境 ---
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg())

    print("[INFO] 正在构建物理躯体 (Franka Panda)...")
    robot_cfg = FRANKA_PANDA_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"
    robot = Articulation(cfg=robot_cfg)

    # 挂载摄像头 (注意：去掉了 device 参数，使用默认的 numpy 输出)
    camera_cfg = CameraCfg(
        prim_path="/World/Robot/top_camera",
        update_period=0.0,
        height=480, width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(1.0, 0.0, 1.0), rot=(0.707, 0.0, 0.707, 0.0))
    )
    camera = Camera(cfg=camera_cfg)

    # --- B. 加载 ACT 大脑 (在环境初始化后安全加载) ---
    print(f"[INFO] 正在唤醒 ACT 神经网络: {args_cli.model_path}")
    policy = ACTPolicy.from_pretrained(args_cli.model_path).to(device)
    policy.eval()
    print("[INFO] 脑机接口准备就绪！")

    sim.reset()
    print("=========================================================")
    print("🚀 仿真启动：ALOHA 意图 -> Franka 躯体")
    print("=========================================================")

    for _ in range(10): sim.step()
    
    with torch.inference_mode():
        while simulation_app.is_running():
            # # 1. 采集视觉 (从 numpy 复制到 PyTorch，这步是安全的)
            # img_data = camera.data.output["rgb"]
            # # 使用 clone() 断开与底层 C++ 缓冲区的关联，防止互相锁死
            # img_tensor = torch.from_numpy(img_data.copy()).to(device).permute(2, 0, 1).unsqueeze(0).float() / 255.0

            # 1. 采集视觉 (此时 img_data 已经是 PyTorch Tensor)
            img_data = camera.data.output["rgb"]
            
            # 使用 clone() 安全复制显存，转为浮点数并归一化
            img_tensor = img_data.clone().to(device).float() / 255.0
            
            # Isaac Lab 默认输出带 Batch 维度的张量 [1, H, W, C]
            # ACT 模型需要 [1, C, H, W]，所以进行维度置换
            if img_tensor.ndim == 4:
                img_tensor = img_tensor.permute(0, 3, 1, 2)
            else:
                # 兼容处理：如果没有 batch 维度 [H, W, C]
                img_tensor = img_tensor.permute(2, 0, 1).unsqueeze(0)
            
            # 2. 状态映射器
            franka_pos = robot.data.joint_pos 
            padded_state = torch.zeros((1, 14), device=device)
            padded_state[0, :9] = franka_pos[0, :9]
            
            lerobot_obs = {
                "observation.images.top": img_tensor,
                "observation.state": padded_state
            }

            # 3. 大脑推理
            action_output = policy.select_action(lerobot_obs)
            
            # 兼容性处理：如果模型返回 3D [Batch, Chunk, Dim]，取第一帧；如果返回 2D [Batch, Dim]，直接用
            if action_output.ndim == 3:
                target_action = action_output[:, 0, :] 
            else:
                target_action = action_output

            # 4. 动作映射器
            # 截断大脑输出，只把前 9 个指令发给 Franka 的电机
            franka_action = target_action[:, :9]
            
            # 确保传递给底层物理引擎的是连续内存分布的浮点张量
            robot.set_joint_position_target(franka_action.contiguous())

            # 5. 驱动物理引擎
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt=sim_cfg.dt)
            camera.update(dt=sim_cfg.dt)

    simulation_app.close()

if __name__ == "__main__":
    main()

"""
./isaaclab.sh -p run_act_franka.py --model_path /Developer/aloha_act_last/last/pretrained_model --enable_cameras
"""