import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="ACT Policy with Aligned Scenario")
parser.add_argument("--model_path", type=str, required=True, help="ACT 模型路径")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import Camera, CameraCfg
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG
from lerobot.policies.act.modeling_act import ACTPolicy

def main():
    device = args_cli.device
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg())

# =========================================================
    # 💥 核心修改一：场景对齐 (加入真实物理碰撞与重力)
    # =========================================================
    print("[INFO] 正在构建带物理碰撞的仿真场景...")
    
    # 1. 生成桌子 (添加静态碰撞体，不再是幽灵)
    table_cfg = sim_utils.CuboidCfg(
        size=(0.4, 0.8, 0.4),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.5, 0.4), roughness=0.3), 
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5), # 赋予摩擦力
        collision_props=sim_utils.CollisionPropertiesCfg(), # 💥 开启物理碰撞！
    )
    # 将桌子固定在空间中
    table_cfg.func("/World/Table", table_cfg, translation=(0.5, 0.0, 0.2))

    # 2. 生成红色积木 (添加碰撞体和刚体动力学)
    cube_cfg = sim_utils.CuboidCfg(
        size=(0.04, 0.04, 0.04),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1), metallic=0.2), 
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8),
        collision_props=sim_utils.CollisionPropertiesCfg(), # 💥 开启物理碰撞！
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),     # 💥 开启重力与物理受力！
        mass_props=sim_utils.MassPropertiesCfg(mass=0.05),  # 设定积木重量为 50克
    )
    # 将积木放在桌面上方一点点，让它自然掉落在桌面上
    cube_cfg.func("/World/Cube", cube_cfg, translation=(0.5, 0.0, 0.45))

    # =========================================================
    
    print("[INFO] 正在构建物理躯体 (Franka Panda)...")
    robot_cfg = FRANKA_PANDA_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"
    robot = Articulation(cfg=robot_cfg)

    # =========================================================
    # 💥 核心修改二：摄像机视角对齐 (模拟 ALOHA 的 Top 视角)
    # =========================================================
    camera_cfg = CameraCfg(
        prim_path="/World/Robot/top_camera",
        update_period=0.0,
        height=480, width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        # 调整：将摄像机挂在较高处 (z=0.8)，稍微向前 (x=0.3)，并且镜头向下俯视
        offset=CameraCfg.OffsetCfg(pos=(0.3, 0.0, 0.8), rot=(0.707, 0.0, 0.707, 0.0)) 
    )
    camera = Camera(cfg=camera_cfg)

    print(f"[INFO] 正在唤醒 ACT 神经网络: {args_cli.model_path}")
    policy = ACTPolicy.from_pretrained(args_cli.model_path).to(device)
    policy.eval()

    sim.reset()
    print("=========================================================")
    print("🚀 场景对齐测试：含有桌子、积木和标准俯视角的仿真")
    print("=========================================================")

    for _ in range(10): sim.step()
    
    with torch.inference_mode():
        while simulation_app.is_running():
            img_data = camera.data.output["rgb"]
            img_tensor = img_data.clone().to(device).float() / 255.0
            
            if img_tensor.ndim == 4:
                img_tensor = img_tensor.permute(0, 3, 1, 2)
            else:
                img_tensor = img_tensor.permute(2, 0, 1).unsqueeze(0)
            
            franka_pos = robot.data.joint_pos 
            padded_state = torch.zeros((1, 14), device=device)
            padded_state[0, :9] = franka_pos[0, :9]
            
            lerobot_obs = {
                "observation.images.top": img_tensor,
                "observation.state": padded_state
            }

            action_output = policy.select_action(lerobot_obs)
            if action_output.ndim == 3:
                target_action = action_output[:, 0, :] 
            else:
                target_action = action_output

            franka_action = target_action[:, :9]
            robot.set_joint_position_target(franka_action.contiguous())

            robot.write_data_to_sim()
            sim.step()
            robot.update(dt=sim_cfg.dt)
            camera.update(dt=sim_cfg.dt)

    simulation_app.close()

if __name__ == "__main__":
    main()

"""
(base) lkk@rtx5090:/Developer/omniverselab$ cp IsaacSim/run_act_aligned.py ../IsaacLab/

(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p run_act_aligned.py --model_path /Developer/aloha_act_last/last/pretrained_model --enable_cameras

"""