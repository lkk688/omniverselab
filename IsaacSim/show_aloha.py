import argparse
from omni.isaac.lab.app import AppLauncher

# 1. 初始化 Isaac Sim 引擎
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. 导入场景和 Aloha 资产
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab_assets.aloha import ALOHA_CFG

def main():
    # 生成基础场景（地面、网格、基础光照）
    cfg = sim_utils.SceneEntityCfg()
    sim_utils.design_scene(cfg)
    
    # 召唤 Aloha 机器人！
    print("[INFO] 正在从资产库中加载 ALOHA 双臂机器人...")
    aloha_cfg = ALOHA_CFG.copy()
    aloha_cfg.prim_path = "/World/Aloha"
    aloha = Articulation(cfg=aloha_cfg)
    
    # 初始化物理引擎
    sim_utils.reset_simulation()
    
    print("--- RTX 5090 渲染已就绪，按 Ctrl+C 退出 ---")
    
    # 保持窗口运行的死循环
    while simulation_app.is_running():
        aloha.write_data_to_sim()
        sim_utils.step_simulation()
        aloha.update(dt=0.01)

if __name__ == "__main__":
    main()
