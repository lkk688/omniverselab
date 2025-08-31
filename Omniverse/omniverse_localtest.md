
## Course: An Introduction to Developing With NVIDIA Omniverse
[link](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-11+V1)

An Omniverse application is defined by a .kit file, a text file that references all the application’s extensions and settings.

The Kit App Template repo is located at
https://github.com/NVIDIA-Omniverse/kit-app-template

```bash
(py312) lkk@lkk-intel13:~/Developer$ git clone https://github.com/NVIDIA-Omniverse/kit-app-template
(py312) lkk@lkk-intel13:~/Developer$ cd kit-app-template/
(py312) lkk@lkk-intel13:~/Developer/kit-app-template$ ls
CHANGELOG.md  PRODUCT_TERMS_OMNIVERSE  repo.bat   repo_tools.toml  tools
LICENSE       readme-assets            repo.sh    SECURITY.md
premake5.lua  README.md                repo.toml  templates
```

In this step, you will use one of the tools included with Kit to create a basic application.
```bash
(py312) lkk@lkk-intel13:~/Developer/kit-app-template$ ./repo.sh template new
Creating packman packages cache at /home/lkk/.cache/packman
Fetching python@3.10.17-nv1-linux-x86_64.tar.gz from bootstrap.packman.nvidia.com ...
Unpacking python
Fetching packman-common@7.29.zip from bootstrap.packman.nvidia.com ...
Unpacking ...
Package successfully installed to /home/lkk/.cache/packman/packman-common/7.29
Pip installing dependencies from /home/lkk/.cache/packman/chk/repo_man/1.84.7/tools/uv/uv-requirements.txt to /home/lkk/.cache/packman/chk/uv_deps/50fa2d6354593bc84586cb1856af2077/linux-x86_64/3.10.17...
By downloading or using the software and materials provided, you agree to the governing terms:

The software and materials are governed by the NVIDIA Software License Agreement and the Product-Specific Terms for NVIDIA Omniverse.

? Do you accept the governing terms? Yes
[CTRL+C to Exit]
? Select what you want to create with arrow keys ↑↓: Application>
? Select desired template with arrow keys ↑↓: [kit_base_editor]: Kit Base Editor
 
? Enter name of application .kit file [name-spaced, lowercase, alphanumeric]: my
_company.my_editor
? Enter application_display_name: My Editor
? Enter version: 0.1.0
Application 'my_company.my_editor' created successfully in /home/lkk/Developer/kit-app-template/source/apps/my_company.my_editor.kit

The application template you have selected has optional linked applications and layers.
? Do you want to add application layers? No

(py312) lkk@lkk-intel13:~/Developer/kit-app-template$ ls source/apps/
my_company.my_editor.kit
```

Note that the contents of the .kit file are textual, making it straightforward to read and edit. The .kit file is basically a manifest of what you want to pull from the Omniverse ecosystem to use in your application. Everything listed in the .kit file is a building block for the application.

To be able to run the application, you need to build it first. The build process compiles your application and its extensions, preparing them for launch.

```bash
(py312) lkk@lkk-intel13:~/Developer/kit-app-template$ ./repo.sh build
....
BUILD (RELEASE) SUCCEEDED (Took 322.40 seconds)

(py312) lkk@lkk-intel13:~/Developer/kit-app-template$ ./repo.sh launch
```

Windows:
```bash
lkk68@NEWALIENWARE D:\Developer>git clone https://github.com/NVIDIA-Omniverse/kit-app-template
lkk68@NEWALIENWARE D:\Developer>cd kit-app-template
lkk68@NEWALIENWARE D:\Developer\kit-app-template>.\repo.bat template new
lkk68@NEWALIENWARE D:\Developer\kit-app-template>.\repo.bat build
.....
>>> VS Code setup. Writing: D:\Developer\kit-app-template\_build\windows-x86_64\release\setup_python_env.bat
>>> VS Code setup. Writing: D:\Developer\kit-app-template\_build\windows-x86_64\release\site\sitecustomize.py
>>> VS Code setup. Writing: D:/Developer/kit-app-template/.vscode/settings.json
BUILD (RELEASE) SUCCEEDED (Took 269.23 seconds)

.\repo.bat launch
```