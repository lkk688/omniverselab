# Digital Twin in CMPE249: Intelligent Autonomous Systems

## Module 4: Digital Twinning, Imitation Learning, and Sim-to-Real Transfer

- Instructor: Kaikai Liu
- Duration: 4 Weeks (8 Lectures + 1 Capstone Project)
- Prerequisites: Python, Introduction to Deep Learning, Basic ROS2 concepts

## Module Overview

This advanced module integrates NVIDIA Omniverse Isaac Sim with modern imitation learning using Hugging Face LeRobot. You will bridge simulation and reality by constructing a Digital Twin workflow. The focus is an end-to-end pipeline:

- Author photorealistic, physics-accurate environments and robots in Isaac Sim (USD-first)
- Teleoperate in simulation to collect expert demonstrations with physical controllers
- Curate multi-modal datasets suitable for imitation learning and foundation models
- Fine-tune policies (e.g., ACT, Diffusion) on task-specific data
- Deploy models back into simulation for evaluation and iterate toward sim-to-real transfer

References: see [OpenUSD Foundations](openusd_foundations.md), [OpenUSD Applied](openusd_applied.md), and [Omniverse Kit](omniverse_kit.md) for USD scene authoring and Kit workflows.

Example Student Demo Previews (final project exemplars):

[![Student Demo 1 – Teleoperation + Imitation Learning](https://drive.google.com/thumbnail?id=1Pug3rc_eojC1vSTjfSJugYlq-OljYaCm&sz=w1280)](https://drive.google.com/file/d/1Pug3rc_eojC1vSTjfSJugYlq-OljYaCm/view?usp=sharing)

[![Student Demo 2 – Digital Twin Deployment](https://drive.google.com/thumbnail?id=1ntHsYPGK7SCwgdBlOAo_YxgX1r2bIwMr&sz=w1280)](https://drive.google.com/file/d/1ntHsYPGK7SCwgdBlOAo_YxgX1r2bIwMr/view?usp=sharing)

Each preview image links to the full Google Drive video. These demos illustrate expected outcomes: end-to-end teleop data collection, dataset curation, policy training, and deployment in the Digital Twin.

## Learning Objectives

By the end of this module, students will be able to:

- Architect a Digital Twin: build physics-accurate robotic environments with calibrated sensors (LiDAR, RGB-D)
- Implement teleoperation pipelines: map PS5 DualSense inputs and real-arm joint states to virtual robots
- Engineer datasets: record synchronized observations and actions; clean, label, and structure for LeRobot
- Train foundation policies: fine-tune ACT and Diffusion-based models on task-specific demonstrations
- Evaluate in simulation: measure success rate, time-to-completion, and collisions under domain randomization
- Plan sim-to-real transfer: identify gaps, design adaptation strategies, and safety-check deployments

## Lecture Outline & Syllabus

### Week 1: The Environment & The Body (Isaac Sim Basics)

#### Lecture 4.1: Introduction to Omniverse & USD

Understanding the Universal Scene Description (USD) format.

Importing URDFs (Universal Robot Description Format) into Isaac Sim.

Rigging the robot: Articulation Roots, Joints, and Drives.

Lab Activity: Importing a Mobile Manipulator (e.g., Franka Emika or custom wheeled robot) into a warehouse scene.

Lab Deliverables:
- USD stage with articulated robot and base environment
- Attached sensors (RGB-D, LiDAR) validated via Kit viewport and simple OmniGraph
- Screenshot or short clip demonstrating sensor outputs

#### Lecture 4.2: Sensorization & Graph Control

Attaching Sensors: RTX Lidar, RGB-D Cameras, and IMUs.

OmniGraph Basics: Visual scripting for sensor publishing.

Ground Truth vs. Noisy Data: Configuring sensor noise models for realism.

Lab Checklist:
- Create an OmniGraph pipeline for publishing camera and LiDAR frames
- Enable noise models and compare outputs against ground truth labels
- Document sensor calibration parameters (FOV, resolution, extrinsics)

Technical Details:
- Camera intrinsics: choose focal length to match target FOV; verify projection against known checkerboard/Charuco pattern in-sim
- Extrinsics: fix sensor mounts on the robot link with USD Xform; record transform tree for dataset metadata
- RTX LiDAR: configure horizontal/vertical resolution, max range, and material reflectance; validate point cloud density and noise
- OmniGraph: use Sensor → Writer nodes to publish frames; ensure consistent timestamping via simulation time

### Week 2: The Digital Twin Interface (Teleoperation)

#### Lecture 4.3: Controller Mapping (The Human-in-the-Loop)

Interfacing hardware with Python (evdev, inputs, or ROS2 Joy).

Mapping Strategy:

Locomotion: Mapping PS5 analog sticks to differential/holonomic drive commands.

Manipulation: Introduction to Inverse Kinematics (IK) solvers in Isaac (Lula/RMPflow).

#### Lecture 4.4: Physical-to-Digital Bridge (The "Twin" Aspect)

Concept: The Physical "Leader" Arm and the Virtual "Follower."

Streaming joint states from a physical arm (e.g., widowX, xArm) into Isaac Sim via TCP/IP or ROS2.

Synchronization challenges: Latency, frequency matching, and safety limits.

Practical Notes:
- Use `pygame` for PS5 input capture; consider ROS2 `joy` for ROS-native workflows
- Log controller events alongside simulation timestamps for dataset alignment
- Implement basic safety guards (rate limiting, joint bounds) during teleop

Teleop Mapping Details:
- DualSense axes (typical): left stick (x: axis 0, y: axis 1), right stick (x: axis 3, y: axis 4); triggers as analog buttons (L2/R2)
- Base control: map left stick to linear/angular velocities; apply deadzone and smoothing filters
- Arm control: use right stick for EE position deltas; combine with shoulder buttons for mode switching (position vs. orientation)
- IK & control: prefer Lula/RMPflow for smooth joint targets; clamp velocities and acceleration; enforce joint limit safety

### Week 3: Imitation Learning with Hugging Face LeRobot

#### Lecture 4.5: Data Collection for Behavioral Cloning

Defining the Task: Pick-and-Place or Navigation.

The Dataset Format: Recording observation.images, observation.state, and action (joint velocities/positions) at 30Hz-50Hz.

Using the LeRobot dataset structure (Hugging Face Hub standards).

Dataset Schema (recommended):
- `observation.images`: resized RGB (e.g., 320x240), optionally depth
- `observation.state`: joint positions/velocities, end-effector pose
- `action`: joint targets or velocity commands at 30–50 Hz
- `meta`: timestamps, episode ids, task labels

Dataset Engineering Details:
- File structure: episodes segmented (e.g., `episode_0001/frames`, `episode_0001/actions.npy`, `meta.json`)
- Time sync: use simulation clock; align image timestamps with action timestamps; store frequency and unit in metadata
- Compression: PNG or JPEG for images; consider chunked arrays (`.npz`/Arrow) for states/actions
- Quality control: remove outlier frames; ensure consistent action scaling; document normalization

#### Lecture 4.6: Model Architectures & Fine-Tuning

Introduction to LeRobot: Library structure and pre-trained models.

Theory: ACT (Action Chunking with Transformers) and Diffusion Policies.

Fine-tuning: Taking a foundation model trained on large datasets (e.g., Open X-Embodiment) and adapting it to our specific Isaac Sim task.

Training Tips:
- Start with ACT for temporally coherent actions; compare with Diffusion for robustness
- Normalize observations (images/state) consistently across train and eval
- Use modest horizons first (8–16) and increase as stability improves
- Monitor success metrics, not just loss; add curriculum by randomizing object positions
- Hardware: ensure GPU memory headroom; enable mixed precision where supported

### Week 4: Deployment & Evaluation

#### Lecture 4.7: Inference & Evaluation in the Loop

Loading the trained PyTorch model.

Closing the loop: Feeding live Isaac Sim camera frames to the model $\rightarrow$ Predicting actions $\rightarrow$ Applying actions to the Sim robot.

Designing Evaluation Metrics: Success rate, time-to-completion, collision checks.

Evaluation Extensions:
- Domain randomization sweeps (textures, lighting, dynamics) to test robustness
- Ablation studies comparing ACT vs. Diffusion policies and sensor subsets
- Logging and visualization of trajectories for qualitative analysis

Evaluation Protocol:
- Define task success criteria (e.g., object grasped within N seconds, minimal collisions)
- Run multiple seeds and initializations; report mean ± std for metrics
- Record per-episode traces (images, actions, states) for post-hoc analysis
- Compare policies under identical randomization settings to isolate effects

#### Lecture 4.8: The Path to Sim-to-Real (Advanced)

Domain Randomization: Varying textures, lighting, and physics properties in Isaac to prevent overfitting.

Strategy for transferring the LeRobot policy to the physical hardware.

Sim-to-Real Checklist:
- Calibrate real sensors to match simulated intrinsics/extrinsics
- Apply domain adaptation (style augmentation, feature normalization) to bridge sim-to-real
- Enforce safety: rate limiting, emergency stop, workspace constraints
- Incremental deployment: dry-run without actuation, then low-power tests, then full autonomy

Assessment & Grading:
- Labs (Weeks 1–3): 30%
- Capstone project (pipeline completeness, reproducibility): 45%
- Final evaluation report (metrics, analysis, video demo): 20%
- Participation (discussions, code reviews): 5%

## Major Course Assignment: "The Digital Puppeteer"

Objective: Create an autonomous clean-up robot pipeline.
Hardware: PC with NVIDIA RTX GPU, PS5 Controller, (Optional) Desktop Robotic Arm.

Phase 1: Scene & Robot Setup (Isaac Sim)

Create a "Tabletop" environment in Isaac Sim with 3 random objects (cubes/cans).

Import a mobile manipulator robot.

Deliverable: A USD file where the robot can be controlled via keyboard, and sensors (Camera/Lidar) are visualizing data.

Phase 2: The Teleop & Data Collector

Write a Python script teleop_collect.py.

Locomotion: Bind PS5 Left Analog Stick to robot base velocity.

Manipulation:

Option A (Sim-Only): Bind PS5 Right Stick to End-Effector IK target.

Option B (Digital Twin): Connect physical arm USB. Read joint angles. Apply these angles directly to the Isaac Sim robot joints (ignoring physics/collision for the "ghost" arm, or using PD control for the physics arm).

Recording: Implement a "Record" button (PS5 'X' button). When held, save synchronized frames (320x240 resized) and joint positions to a local folder structured for LeRobot.

Deliverable: A dataset of 50 successful "pick up object" demonstrations.

Milestones & Rubric:
- M1 (Environment & Sensors): robot imported, sensors streaming (10%)
- M2 (Teleop & Recorder): controller mapping, synchronized logging (15%)
- M3 (Dataset & Training): formatted dataset, training run completes (20%)
- M4 (Inference & Evaluation): autonomous run with metrics + video (25%)
- Report: method, results, lessons learned, next steps (30%)

Phase 3: Training with LeRobot

Convert your raw dataset to the LeRobot / Hugging Face dataset format (.arrow or standard folders).

Use the provided Colab/Local notebook to fine-tune a Diffusion Policy.

Training configurations:

Batch size: 32

Epochs: 500

Horizon: 16 steps (Action Chunking).

Deliverable: Training loss graphs and the saved model weights (policy.pt).

Phase 4: Deployment (Inference)

Write eval_policy.py.

Load the trained policy.

Reset the Isaac Sim environment.

Run the robot autonomously. The script should capture camera data from Sim, pass it to the model, and execute the returned actions.

Deliverable: A video recording of the robot successfully performing the task autonomously in Isaac Sim.

Submission Guidelines:
- Repository: include scripts (`teleop_collect.py`, `eval_policy.py`), configs, and README
- Data: sample episode (images + actions) and dataset conversion script/notebook
- Report: PDF with methodology, metrics tables, and links to demo video

## Technical Stack & Implementation Details

### Software Requirements

- OS: Ubuntu 20.04/22.04
- Simulation: NVIDIA Isaac Sim 4.0+
- ML Framework: PyTorch, Hugging Face LeRobot, Diffusers
- Control: pygame (for PS5), rospy/rclpy (optional, for ROS bridge)

### Setup Checklist
- Isaac Sim 4.0+ installed and validated (see `docs/isaac_sim.md`)
- Python environment with PyTorch, LeRobot, Diffusers, and required drivers
- PS5 controller paired and recognized (`pygame` or `ros2 joy`)
- Optional ROS2 bridge configured for joint streaming

Domain Randomization & Synthetic Data:
- Use Omniverse Replicator to randomize materials, lighting, and physics parameters
- Vary textures, backgrounds, object positions, and physical properties across episodes
- Export synthetic datasets with perfect ground truth for benchmarking and pretraining

### Resources
- USD & Kit: [OpenUSD Foundations](openusd_foundations.md), [OpenUSD Applied](openusd_applied.md), [Omniverse Kit](omniverse_kit.md)
- Isaac Sim: [Isaac Sim](isaac_sim.md)
- ROS 2 Integration: [ROS 2 Integration](ros2_integration.md)

### Important Reference Links
- NVIDIA Isaac Sim Docs: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Omniverse Replicator (synthetic data): https://developer.nvidia.com/omniverse/replicator
- OmniGraph Overview: https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/overview.html
- USD (Pixar): https://graphics.pixar.com/usd/docs/index.html
- Hugging Face LeRobot: https://github.com/huggingface/lerobot
- Open X-Embodiment (foundation dataset): https://arxiv.org/abs/2306.08764
- ACT (Action Chunking with Transformers): https://arxiv.org/abs/2304.13705
- Diffusion Policy for Robot Control: https://arxiv.org/abs/2303.01469
- ROS 2 Docs: https://docs.ros.org/en/
- pygame DualSense input: https://www.pygame.org/docs/
- RMPflow (Isaac): https://docs.omniverse.nvidia.com/isaacsim/latest/robotics_isaac/motion_generation.html
- Lula IK: https://docs.omniverse.nvidia.com/isaacsim/latest/robotics_isaac/ik_solver.html