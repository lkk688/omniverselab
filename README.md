# omniverselab
A curated lab for exploring the NVIDIA Omniverse ecosystem — including OpenUSD, Omniverse Kit, Isaac Sim, ROS 2 integration, and hardware-in-the-loop (HIL) simulation.

## 🔧 Modules Covered

- **OpenUSD**: Learn the core of Omniverse's scene representation format.
- **Omniverse Kit**: Build powerful extensions and UI workflows.
- **Kit App Streaming**: Enable remote interaction through web-based streaming.
- **Isaac Sim**: High-fidelity robotics simulation with physics.
- **ROS 2 Integration**: Bridge simulation and robotics software.
- **HIL Simulation**: Integrate Jetson and real hardware with Isaac Sim.

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

