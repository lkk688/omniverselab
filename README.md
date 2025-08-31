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

