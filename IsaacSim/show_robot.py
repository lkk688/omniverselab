import argparse
from isaaclab.app import AppLauncher

# 1. 启动 Isaac Lab 引擎
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. 导入最新版仿真组件
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG

def main():
    # 3. 创建仿真上下文 (控制物理引擎的步长和设备)
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # 4. 生成基础场景 (地面和穹顶光)
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg())
    
    # 5. 召唤 Franka 机械臂
    print("[INFO] 正在将 Franka 机械臂加载到显存中...")
    robot_cfg = FRANKA_PANDA_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"
    robot = Articulation(cfg=robot_cfg)
    
    # 6. 初始化并重置物理引擎
    sim.reset()
    print("--- RTX 5090 渲染完成！请在弹出的黑色窗口中查看。按 Ctrl+C 退出 ---")
    
    # 7. 物理渲染死循环
    while simulation_app.is_running():
        # 推进仿真时间步
        sim.step()

if __name__ == "__main__":
    main()
