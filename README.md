# omniverselab
A curated lab for exploring the NVIDIA Omniverse ecosystem — including OpenUSD, Omniverse Kit, Isaac Sim, ROS 2 integration, and hardware-in-the-loop (HIL) simulation. Document is hosted on: https://lkk688.github.io/omniverselab/

## 🔧 Modules Covered

- **OpenUSD**: Learn the core of Omniverse's scene representation format.
- **Omniverse Kit**: Build powerful extensions and UI workflows.
- **Kit App Streaming**: Enable remote interaction through web-based streaming.
- **Isaac Sim**: High-fidelity robotics simulation with physics.
- **ROS 2 Integration**: Bridge simulation and robotics software.
- **HIL Simulation**: Integrate Jetson and real hardware with Isaac Sim.
- **VM System Check**: Comprehensive cloud VM environment analysis tools.

## 🗺️ Getting Started

1. Clone this repo:
```bash
% git clone https://github.com/lkk688/omniverselab.git
```

2. Setup MkDocs:
```bash
pip install mkdocs mkdocs-material
mkdocs serve
# to build:
mkdocs build
# to deploy:
mkdocs gh-deploy
```

## 🖥️ VM System Check Tools

Comprehensive system analysis tools designed for cloud VM environments, especially NVIDIA DLI cloud instances. These tools require **no sudo privileges** and provide detailed information about your system's capabilities.

### Quick Start
```bash
# Python version (recommended - comprehensive analysis)
python3 vm_system_check.py

# Shell version (lightweight - basic checks)
bash vm_system_check.sh

# Interactive launcher (choose your preferred tool)
bash run_system_check.sh
```

### What These Tools Check
- ✅ **System Info**: OS, architecture, uptime, container detection
- ✅ **CPU Details**: Model, cores, load average
- ✅ **GPU Analysis**: NVIDIA GPU detection, VRAM, driver versions
- ✅ **Memory Usage**: RAM capacity, availability, usage percentages
- ✅ **Disk Space**: Storage usage with status indicators
- ✅ **Network**: Local/public IP, connectivity, geographic location
- ✅ **SSH Access**: Daemon status, port accessibility, key detection
- ✅ **VS Code Web**: code-server detection, web IDE availability

### Perfect for Cloud Environments
- 🎯 **NVIDIA DLI Cloud Instances**: GPU detection and CUDA environment checking
- 🎯 **AWS EC2, Google Cloud, Azure**: General cloud VM analysis
- 🎯 **Remote Development**: SSH and web-based IDE setup verification
- 🎯 **Educational Labs**: Quick system capability assessment

For detailed documentation, see [README_VM_SYSTEM_CHECK.md](README_VM_SYSTEM_CHECK.md).

## OpenUSD
The OpenUSD source code is located at the [OpenUSD Github repository](https://github.com/PixarAnimationStudios/OpenUSD). You can clone the repository and follow the repository's build instructions to build OpenUSD from the source. Alternatively, NVIDIA provides pre-built binaries for Windows and Linux from [here](https://developer.nvidia.com/usd?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.usd_resources%3Adesc%2Ctitle%3Aasc&hitsPerPage=6#section-getting-started). If you just need to use the OpenUSD Python API, you can install usd-core directly from PyPI, i.e., `pip install usd-core`.

### OpenUSD Visualization Tools
This repository includes custom visualization utilities for OpenUSD files:

- **`openusd/myusddisplay.py`**: A comprehensive USD to GLB converter and viewer that supports multiple display methods:
  - **Web Browser Display**: Interactive 3D viewing using Google's model-viewer with full camera controls, lighting, and material inspection
  - **macOS Quick Look**: Native preview integration for basic 3D model viewing
  - **Jupyter Notebook**: Inline 3D model display for educational and development workflows
  - **Base64 Encoding**: CORS-free web display by embedding GLB data directly in HTML

- **`openusd/test_usd_display.py`**: Test script demonstrating the visualization functionality with error handling and debugging features for various USD/USDZ file formats.

These tools bridge the gap between USD's powerful scene description capabilities and accessible 3D visualization, making it easier to preview and share OpenUSD content across different platforms and environments.

### Pre-built binaries for Windows
In Windows, Right-click the `usd.py311.windows-x86_64.usdview.release-0.25.05-25f3d3d8.zip` → “Extract All…” to extract the contents to a folder. Open Command Prompt or PowerShell, run the setup script:

To launch `usdview`, execute the `%USDROOT%\scripts\usdview_gui.bat` script from Explorer, or invoke the executable from a Batch prompt using:
```bat
(base) PS E:\Shared\usd.py311.windows-x86_64.usdview.release-0.25.05-25f3d3d8> .\scripts\usdview.bat .\share\usd\tutorials\traversingStage\HelloWorld.usda
```

### Pre-built binaries for Linux
```bash
(py312) lkk@lkk-intel13:~/Developer$ unzip usd.py311.manylinux_2_35_x86_64.usdview.release@0.25.05-25f3d3d8.zip -d ./openusd/
```

To launch `usdview`, execute:
```shell
(py312) lkk@lkk-intel13:~/Developer/openusd$ ./scripts/usdview.sh ./share/usd/tutorials/traversingStage/HelloWorld.usda
```

To execute `usdcat` and inspect the contents of a USD stage, invoke the executable from a bash terminal using:
```shell
(py312) lkk@lkk-intel13:~/Developer/openusd$ ./scripts/usdcat.sh ./share/usd/tutorials/convertingLayerFormats/Sphere.usd
Activated USD python/tools from /home/lkk/Developer/openusd
#usda 1.0

def Sphere "sphere"
{
}
```

If you wish to configure a bash terminal with environment variables defined to use OpenUSD tools without the need for>
```shell
(py312) lkk@lkk-intel13:~/Developer/openusd$ chmod +x ./scripts/set_usd_env.sh
(py312) lkk@lkk-intel13:~/Developer/openusd$ . ./scripts/set_usd_env.sh
Activated USD python/tools from /home/lkk/Developer/openusd
```
This is same to `source ./scripts/set_usd_env.sh`. This will update your shell’s environment with: PATH including openusd/bin; PYTHONPATH pointing to lib/python; LD_LIBRARY_PATH for shared libraries. You can add this to `~/.bashrc` to make it persistent: `source ~/Developer/openusd/scripts/set_usd_env.sh`

This will allow you to invoke the provided compiled tools and libraries located in `$USDROOT/bin`, without having to script
```bash
(py312) lkk@lkk-intel13:~/Developer/openusd$ usdcat ./share/usd/tutorials/convertingLayerFormats/Sphere.usd
#usda 1.0

def Sphere "sphere"
{
}
```

The Python interpreter is also setup by the `set_usd_env.sh` and the Python interpreter will then resolve OpenUSD imports, allowing OpenUSD APIs to be used in the prompt:
```bash
(py312) lkk@lkk-intel13:~/Developer/openusd$ python
Python 3.11.11 (tags/v3.11.11-dirty:d03b868, Feb  7 2025, 19:34:23) [GCC 7.3.1 20180303 (Red Hat 7.3.1-5)] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from pxr import Usd, UsdGeom
```
Save the following code to a python file
```python
from pxr import Usd, UsdGeom
stage = Usd.Stage.CreateInMemory()
cube = UsdGeom.Cube.Define(stage, "/myCube")
cube.GetSizeAttr().Set(3)
print(stage.ExportToString())
```
Run the python file:
```bash
(py312) lkk@lkk-intel13:~/Developer/openusd$ nano pytest.py
(py312) lkk@lkk-intel13:~/Developer/openusd$ python pytest.py 
#usda 1.0
(
    doc = """Generated from Composed Stage of root layer 
"""
)

def Cube "myCube"
{
    double size = 3
}
(py312) lkk@lkk-intel13:~/Developer/openusd$ python -c "from pxr import Usd;print(Usd.GetVersion())"
(0, 25, 5)
```


## NVIDIA Omniverse for Developers
[Get Started with Omniverse](https://developer.nvidia.com/omniverse?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.omniverse%3Adesc%2Ctitle%3Aasc&hitsPerPage=6#section-getting-started)
[Documentation](https://docs.omniverse.nvidia.com/index.html)

Develop an OpenUSD-native application from scratch with the Omniverse Kit SDK and developer tooling, including the Omniverse App Streaming API and the legacy Omniverse Launcher (Omniverse Launcher will be deprecated on October 1, 2025).
[Download Omniverse Kit SDK](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/omniverse/collections/kit)

Install NGC CLI from [link](https://org.ngc.nvidia.com/setup/installers/cli), use Mac as an example. Go to: https://ngc.nvidia.com/setup, Log in with your NVIDIA developer account. Generate your API key
```bash
% curl -LO https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/3.164.0/files/ngccli_mac_arm.pkg
% sudo installer -pkg ngccli_mac_arm.pkg -target /usr/local
% export LC_ALL=en_US.UTF-8
% ngc config set #enter api key, 
Enter API key [no-apikey]. Choices: [<VALID_APIKEY>, 'no-apikey']: nvapi-xxxxxxxx
Enter CLI output format type [ascii]. Choices: ['ascii', 'csv', 'json']: ascii
Enter org [no-org]. Choices: ['0529344315741393']: sjsu
Invalid org. Please re-enter.
Enter org [no-org]. Choices: ['0529344315741393']: 0529344315741393
Enter team [no-team]. Choices: ['no-team']: no-team
Enter ace [no-ace]. Choices: ['no-ace']: no-ace
Validating configuration...
Successfully validated configuration.
Saving configuration...
Successfully saved NGC configuration to /Users/kaikailiu/.ngc/config
```
Download Omniverse Kit SDK via ngc command line:
```bash
% ngc registry resource download-version "nvidia/omniverse/kit-sdk-linux:107.3.0"
% ngc registry resource download-version "nvidia/omniverse/kit-sdk-windows:107.3.0"
```

To this sdk to one Linux machine, unzip it, and run the python wrap:
```bash
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ ./python.sh 
Python 3.11.12 (tags/v3.11.12-dirty:da1f6c6, May  6 2025, 19:33:15) [GCC 7.3.1 20180303 (Red Hat 7.3.1-5)] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from pxr import Usd
>>> stage = Usd.Stage.CreateNew('test.usda');
>>> print(stage)
Usd.Stage.Open(rootLayer=Sdf.Find('/home/lkk/Developer/kit-sdk-linux_v107.3.0/test.usda'), sessionLayer=Sdf.Find('anon:0x213fe1c0:test-session.usda'), pathResolverContext=Ar.ResolverContext(Ar.DefaultResolverContext(['/home/lkk/Developer/kit-sdk-linux_v107.3.0'])))
>>> 
```
**python.sh** is not a standard Python interpreter — it’s a wrapper around a pre-configured Python runtime with many Omniverse-specific binary modules (e.g., pxr, omni, carb). These modules are C++-backed, compiled against a very specific environment — often not compatible with Conda or arbitrary Python versions.

Use Kit’s environment inside Jupyter notebooks. Run with ./python.sh, not system python. This ensures you use Kit’s embedded Python, not Conda’s.
```bash
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ ./python.sh -m pip install ipykernel
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ ./python.sh -m ipykernel install --user --name kit-sdk-107.3.0 --display-name "Omniverse Kit Python"
Installed kernelspec kit-sdk-107.3.0 in /home/lkk/.local/share/jupyter/kernels/kit-sdk-107.3.0
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ pip install jupyter jupyter_client 
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ jupyter kernelspec list
Available kernels:
  python3            /home/lkk/miniconda3/envs/py312/share/jupyter/kernels/python3
  kit-sdk-107.3.0    /home/lkk/.local/share/jupyter/kernels/kit-sdk-107.3.0
  mycondapy310       /home/lkk/.local/share/jupyter/kernels/mycondapy310
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ ./python.sh -m pip install typing_extensions
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
#access it in your local browser: http://10.31.81.235:8888/lab?token=3bbb4f078b8e130f395e64560be4021c2cd59c95955a4c99
```

Error fixes:
```bash
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ jupyter kernelspec list
Available kernels:
  python3            /home/lkk/miniconda3/envs/py312/share/jupyter/kernels/python3
  kit-sdk-107.3.0    /home/lkk/.local/share/jupyter/kernels/kit-sdk-107.3.0
  mycondapy310       /home/lkk/.local/share/jupyter/kernels/mycondapy310
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ cat /home/lkk/.local/share/jupyter/kernels/kit-sdk-107.3.0/kernel.json
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ /home/lkk/Developer/kit-sdk-linux_v107.3.0/python/bin/python3.11 -m pip install typing_extensions --no-cache-dir --force-reinstalll
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ /home/lkk/Developer/kit-sdk-linux_v107.3.0/python/bin/python3.11 -m pip install psutil --no-cache-dir
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ nano ~/.local/share/jupyter/kernels/kit-sdk-107.3.0/kernel.json
#Replace the "argv" line with:
    "argv": [
    "/home/lkk/Developer/kit-sdk-linux_v107.3.0/python/bin/python3",
    "-m",
    "ipykernel_launcher",
    "-f",
    "{connection_file}"
    ],
(py312) lkk@lkk-intel13:~/Developer/kit-sdk-linux_v107.3.0$ jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
#select the "Omniverse Kit Python" kernel in jupyter lab, you can run "from pxr import Usd, UsdGeom" without any problems
```

[Omniverse Kit App Template](https://github.com/NVIDIA-Omniverse/kit-app-template)

# ISAAC Sim Setup
https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html

```bash
conda create -n isaac_lerobot python=3.11 -y
conda activate isaac_lerobot
python -m pip install --upgrade pip
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com


```

```bash
# 1. 设置 ROS 发行版
export ROS_DISTRO=jazzy

# 2. 设置底层通信中间件
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. 将 conda 环境下的 ROS2 库路径添加到系统的动态库查找路径中
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/lkk/miniconda3/envs/isaac_lerobot/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib

# 1. 创建一个 32GB 的交换文件 (会占用一点硬盘空间，但能保命)
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 2. 检查是否成功开启
sudo swapon --show

# note: you can pass the argument "--help" to see all arguments possible.
isaacsim

# experience files can be absolute path, or relative path searched in isaacsim/apps or omni/apps
isaacsim isaacsim.exp.full.kit
```

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
(isaac_lerobot) lkk@rtx5090:/Developer$ cd IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh --install

git clone https://github.com/huggingface/lerobot.git
(isaac_lerobot) lkk@rtx5090:/Developer$ cd lerobot/
(isaac_lerobot) lkk@rtx5090:/Developer/lerobot$ pip install -e ".[aloha,pi]"

pip uninstall transformers -y
pip install git+https://github.com/huggingface/transformers.git@fix/lerobot_openpi

pip install av wandb
```

```bash
pip uninstall torchcodec

(isaac_lerobot) lkk@rtx5090:/Developer/lerobot$ python src/lerobot/scripts/lerobot_dataset_viz.py   --repo-id lerobot/aloha_sim_insertion_human   --episode-index 0

(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ python scripts/tutorials/00_sim/create_empty.py

#use Isaac Lab to train a robot through Reinforcement Learning
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless

(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 

```

simulation control
```bash
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ ./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --teleop_device keyboard --num_demos 1

./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --teleop_device gamepad --num_demos 1

#streaming
./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --teleop_device keyboard --num_demos 100 --livestream 2

./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --teleop_device gamepad --num_demos 100 --livestream 2
```

dual-GPU workstation notes
```bash
# Current local GPU map:
#   GPU 0 = NVIDIA RTX PRO 6000 Blackwell Workstation Edition (busy / reserve)
#   GPU 1 = NVIDIA GeForce RTX 5090 (use for Isaac Lab / Isaac Sim collection)
nvidia-smi --query-gpu=index,name,memory.used,memory.total --format=csv

# Recommended: run IsaacLab on the physical RTX 5090 at cuda:1.
# isaac_auto_collector_v5.py / v6.py forward --device into parse_env_cfg, so
# AppLauncher, PhysX/rendering, and environment tensors use the same GPU.
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v5.py /Developer/IsaacLab/

./isaaclab.sh -p isaac_auto_collector_v5.py --device cuda:1 --enable_cameras --autorun --env lift-ik-rel   # cube lift
./isaaclab.sh -p isaac_auto_collector_v5.py --device cuda:1 --enable_cameras --autorun --env stack-ik-rel  # cube stack

# V6 multi-camera recorder:
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v6.py /Developer/IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_multicam_addons.py /Developer/IsaacLab/

# 2026-05-24 note: isaac_multicam_addons.py uses IsaacLab's wxyz quaternion
# order for camera offsets. Use the fixed path below for training data.
# The older logs/demos_multicam_lift run is structurally valid but had blank
# top/right views before this fix.
./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --list_cams \
  --env lift-ik-rel --cams top,left,right,front,wrist --cam_hw 64,64

./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --autorun \
  --env lift-ik-rel --max_demos 50 --exit_on_done \
  --cams top,left,right,front,wrist --cam_hw 480,640 \
  --save_dir logs/demos_multicam_lift_fixed

# Convert V6 HDF5 episodes to a native LeRobotDataset for ACT/pi0-style training:
PYTHONPATH=/Developer/lerobot/src python /Developer/omniverselab/IsaacSim/convert_isaac_hdf5_to_lerobot.py \
  logs/demos_multicam_lift_fixed \
  --repo_id local/isaac_multicam_lift_fixed \
  --root logs/lerobot_multicam_lift_fixed \
  --task "lift the cube" \
  --image_dtype video \
  --resize_hw 240,320 \
  --overwrite

# Manual demo recording can use the same GPU flag:
./isaaclab.sh -p scripts/tools/record_demos.py --device cuda:1 --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --teleop_device keyboard --num_demos 1

# Alternative hard CUDA mask: physical GPU 1 becomes logical cuda:0.
# Use this only if you intentionally want to hide the Pro 6000 from CUDA.
CUDA_VISIBLE_DEVICES=1 ./isaaclab.sh -p isaac_auto_collector_v5.py --device cuda:0 --enable_cameras --autorun --env lift-ik-rel
```

```bash
# 1. 强制 MuJoCo 使用 EGL 渲染后端
export MUJOCO_GL=egl

# 2. 运行评估命令
python -m lerobot.scripts.lerobot_eval \
  --policy.type act \
  --policy.pretrained_path /Developer/aloha_act_last/last/pretrained_model \
  --env.type aloha \
  --eval.n_episodes 10 \
  --eval.batch_size 1
# outputs/eval/2026-02-23/22-33-10_aloha_act/videos/aloha_0/eval_episode_0.mp4'


# 确保已经设置了离线渲染环境变量，防止 5090 弹窗冲突
export MUJOCO_GL=egl

python -m lerobot.scripts.lerobot_eval \
  --policy.type pi0 \
  --policy.pretrained_path /Developer/aloha_pi0_last/last/pretrained_model \
  --env.type aloha \
  --eval.n_episodes 10 \
  --eval.batch_size 1 \
  --device cuda

python -m lerobot.scripts.lerobot_eval \
  --policy.pretrained_path ./my_local_model \
  --env.type aloha \
  --eval.n_episodes 10 \
  --eval.batch_size 1 \
  --device cuda

python -m lerobot.scripts.lerobot_eval \
  --policy.pretrained_path ./my_pi0_model \
  --env.type aloha \
  --eval.n_episodes 10 \
  --eval.batch_size 1 \
  --device cuda

# 在 Isaac Lab 环境中运行数据收集脚本
python source/standalone/workflows/teleoperation/teleop_se3_agent.py \
  --task Isaac-Repose-Cube-Aloha-v0 \
  --teleop_device keyboard \
  --save_lerobot_dataset True \
  --dataset_dir ./my_custom_dataset
```
