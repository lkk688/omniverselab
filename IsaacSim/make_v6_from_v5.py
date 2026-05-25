#!/usr/bin/env python
"""Generate isaac_auto_collector_v6.py from v5 by applying surgical edits.

Why this exists: v6 reuses ~1400 of v5's 1500 lines. Duplicating them risks
divergence when v5 evolves. So we keep v5 as the source of truth and use
this script to bake in the few multi-camera additions.

Run (once, on either RTX5090 box or H100 — pure Python, no GPU needed):

    cd /Developer/omniverselab/IsaacSim   # (or wherever v5 lives)
    python make_v6_from_v5.py
    # Generates isaac_auto_collector_v6.py in the same directory.

Then copy v6 + addons to your IsaacLab working dir as usual:

    cp isaac_auto_collector_v6.py isaac_multicam_addons.py /Developer/IsaacLab/
    cd /Developer/IsaacLab
    ./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --autorun \\
        --env lift-ik-rel --max_demos 50 --cams top,left,right,front,wrist

Re-run this script any time v5 changes upstream.
"""

from __future__ import annotations

import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
V5 = HERE / "isaac_auto_collector_v5.py"
V6 = HERE / "isaac_auto_collector_v6.py"

if not V5.exists():
    raise SystemExit(f"v5 not found at {V5}")

src = V5.read_text()


# ----------------------------------------------------------------------------
# Edit 1: replace top docstring with v6 docstring
# ----------------------------------------------------------------------------
V6_DOCSTRING = '''"""
Isaac Sim Automatic Data Collection V6 — multi-camera + extrinsics
====================================================================

Generated from V5 by `make_v6_from_v5.py`. DO NOT edit directly —
edit V5 + the multicam_addons module + re-run the generator.

Diff vs V5:
  1. Imports `isaac_multicam_addons` (sibling module).
  2. Adds CLI flags: --cams, --cam_hw, --list_cams.
  3. After parse_env_cfg(), injects additional camera sensors into the scene.
  4. Replaces single-camera LeRobotDemoSaver with multi-camera LeRobotMultiCamSaver.
  5. Per-step capture loops over all selected cameras and records
     obs_t/images_t/action_t plus pose + RGB + extrinsic per frame.
  6. Disables Fabric for this single-env camera recorder so camera readback
     works on a non-default CUDA device such as cuda:1.

H5 output layout (V5-compatible for `obs/images/top` + `obs/state` + `actions`):

    actions                                       [T, 7]
    obs/state                                     [T, J]
    obs/images/<cam>                              [T, H, W, 3]  uint8
    obs/cam_extrinsics_world2cam_cv/<cam>         [T, 4, 4]     float64
    obs/cam_pos_world/<cam>                       [T, 3]        float64
    obs/cam_quat_world_wxyz/<cam>                 [T, 4]        float64
    obs/cam_intrinsics_K/<cam>                    [3, 3]        float64

Run (on RTX5090, in the isaac_lerobot conda env):

    conda activate isaac_lerobot
    cd /Developer/IsaacLab
    cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v6.py .
    cp /Developer/omniverselab/IsaacSim/isaac_multicam_addons.py .
    ./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --autorun \\
        --env lift-ik-rel --max_demos 50 \\
        --cams top,left,right,front,wrist --cam_hw 480,640 \\
        --save_dir logs/demos_multicam_lift

Use `--list_cams` to confirm the scene picked up the cameras you registered
(prints all keys under env.scene.sensors and exits).
"""'''


def _replace_top_docstring(s: str) -> str:
    # Match the first triple-quoted block at the very top of the file.
    m = re.match(r'\s*"""[\s\S]*?"""', s)
    if not m:
        raise RuntimeError("Could not find v5's top docstring to replace.")
    return V6_DOCSTRING + s[m.end():]


# ----------------------------------------------------------------------------
# Edit 2: inject `import isaac_multicam_addons as mc` after `import zmq`
# ----------------------------------------------------------------------------
def _add_addons_import(s: str) -> str:
    needle = "import zmq"
    if needle not in s:
        raise RuntimeError("Couldn't find `import zmq` to anchor addons import.")
    return s.replace(needle,
                     needle + "\nimport isaac_multicam_addons as mc",
                     1)


# ----------------------------------------------------------------------------
# Edit 3: add CLI flags --cams, --cam_hw, --list_cams just before AppLauncher
# ----------------------------------------------------------------------------
NEW_CLI = '''
# >>> multicam: extra flags
parser.add_argument("--cams", type=str, default="top,left,right,front,wrist",
                    help="Comma-separated camera names to capture. "
                         "Available: top, left, right, front, wrist.")
parser.add_argument("--cam_hw", type=str, default="480,640",
                    help="Render H,W per camera (default 480,640).")
parser.add_argument("--list_cams", action="store_true",
                    help="Print env.scene.sensors keys after init and exit.")
'''


def _add_cli_flags(s: str) -> str:
    needle = "AppLauncher.add_app_launcher_args(parser)"
    if needle not in s:
        raise RuntimeError("Couldn't find AppLauncher.add_app_launcher_args.")
    return s.replace(needle, NEW_CLI + "\n" + needle, 1)


# ----------------------------------------------------------------------------
# Edit 4: after `env_cfg = parse_env_cfg(...)` line, register multicam cameras.
#         The change is exactly *one inserted block*.
# ----------------------------------------------------------------------------
def _inject_scene_cfg_call(s: str) -> str:
    # Match the v5 line: `env_cfg = parse_env_cfg(...)` and the two following
    # num_envs/episode_length tweaks. Insert addon call after them.
    pat = re.compile(
        r"(env_cfg\s*=\s*parse_env_cfg\(\s*preset\.task_id"
        r"(?:\s*,\s*device\s*=\s*args_cli\.device)?\s*\)\s*\n"
        r"\s*env_cfg\.scene\.num_envs\s*=\s*1\s*\n"
        r"\s*env_cfg\.episode_length_s\s*=\s*3600\.0\s*\n)"
    )
    if not pat.search(s):
        raise RuntimeError("Couldn't locate parse_env_cfg / num_envs block.")
    inject = (
        "\n    # >>> multicam: inject extra cameras into the scene before env init\n"
        "    cam_names = [c.strip() for c in args_cli.cams.split(',') if c.strip()]\n"
        "    cam_H, cam_W = [int(x) for x in args_cli.cam_hw.split(',')]\n"
        "    mc.add_multicam_to_scene_cfg(env_cfg, cam_names, cam_H, cam_W)\n"
    )
    return pat.sub(r"\1" + inject, s, count=1)


# ----------------------------------------------------------------------------
# Edit 4b: v6 reads camera tensors every step. On this dual-GPU workstation,
#          Isaac's Fabric/USDRT path can hang when camera readback asks for
#          cuda:1 ("GPUs other than cuda:0 are not currently supported").
#          Disable Fabric for this single-env recorder; slower, but reliable.
# ----------------------------------------------------------------------------
def _disable_fabric_for_multicam(s: str) -> str:
    pat = re.compile(
        r"env_cfg\s*=\s*parse_env_cfg\(\s*preset\.task_id\s*,\s*device\s*=\s*args_cli\.device\s*\)"
    )
    replacement = "env_cfg = parse_env_cfg(preset.task_id, device=args_cli.device, use_fabric=False)"
    if not pat.search(s):
        raise RuntimeError("Couldn't locate v6 parse_env_cfg line to disable Fabric.")
    return pat.sub(replacement, s, count=1)


# ----------------------------------------------------------------------------
# Edit 5: after env init, if --list_cams, print + exit. Then replace saver.
# ----------------------------------------------------------------------------
def _replace_saver_init(s: str) -> str:
    # V5 instantiates LeRobotDemoSaver with positional args. Find the exact
    # assignment line and replace with the multicam version. Insert --list_cams
    # check right before it.
    pat = re.compile(
        r"^[ \t]*saver\s*=\s*LeRobotDemoSaver\([\s\S]*?\)\s*\n",
        re.MULTILINE,
    )
    if not pat.search(s):
        raise RuntimeError("Couldn't find LeRobotDemoSaver instantiation.")
    replacement = (
        "    # >>> multicam: --list_cams (early exit) and switch to multi-cam saver\n"
        "    if getattr(args_cli, 'list_cams', False):\n"
        "        print('\\n=== env.scene.sensors keys ===')\n"
        "        if hasattr(env.scene, 'sensors'):\n"
        "            for k in env.scene.sensors:\n"
        "                print(f'  {k}')\n"
        "        env.reset()\n"
        "        env.sim.render()\n"
        "        cam_records = mc.capture_all_cams(env, cam_names)\n"
        "        print('\\n=== capture_all_cams smoke ===')\n"
        "        for c in cam_names:\n"
        "            rec = cam_records.get(c)\n"
        "            if rec is None:\n"
        "                print(f'  {c}: MISSING')\n"
        "            else:\n"
        "                print(f'  {c}: rgb{tuple(rec[\"rgb\"].shape)}')\n"
        "        env.close(); simulation_app.close(); return\n"
        "    _cam_intrinsics = {\n"
        "        c: mc.intrinsic_from_pinhole_cfg(\n"
        "            focal_length_mm=24.0, horizontal_aperture_mm=20.955,\n"
        "            height=cam_H, width=cam_W,\n"
        "        ) for c in cam_names\n"
        "    }\n"
        "    saver = mc.LeRobotMultiCamSaver(\n"
        "        save_dir=args_cli.save_dir,\n"
        "        cam_names=cam_names,\n"
        "        intrinsics_per_cam=_cam_intrinsics,\n"
        "        save_failed=args_cli.save_failed,\n"
        "        task_id=preset.task_id,\n"
        "    )\n"
    )
    # lambda bypasses re.sub's backslash-escape interpretation in `replacement`.
    return pat.sub(lambda _m: replacement, s, count=1)


# ----------------------------------------------------------------------------
# Edit 6: replace v5's post-step single-camera recording with v6's
#         pre-step multi-camera recording. This stores obs_t/images_t paired
#         with action_t, then applies action_t to advance the simulation.
# ----------------------------------------------------------------------------
def _replace_capture_block(s: str) -> str:
    pat = re.compile(
        r"^[ \t]*# Step[ \t]*\n"
        r"^[ \t]*obs,[ \t]*rewards,[ \t]*dones,[ \t]*_,[ \t]*_[ \t]*=[ \t]*env\.step\(final_action\)[ \t]*\n"
        r"^[ \t]*robot_state[ \t]*=[ \t]*robot\.data\.joint_pos\[0\][ \t]*\n"
        r"^[ \t]*img[ \t]*=[ \t]*None[ \t]*\n"
        r"^[ \t]*if hasattr\(env\.scene,[ \t]*['\"]sensors['\"]\) and ['\"]top_camera['\"] in env\.scene\.sensors:[ \t]*\n"
        r"^[ \t]*img[ \t]*=[ \t]*env\.scene\.sensors\[['\"]top_camera['\"]\]\.data\.output\[['\"]rgb['\"]\]\[0\][ \t]*\n"
        r"^[ \t]*saver\.record_step\(robot_state,[ \t]*final_action\[0\],[ \t]*image=img\)[ \t]*\n",
        re.MULTILINE,
    )
    if not pat.search(s):
        raise RuntimeError(
            "Couldn't find v5 step + single-camera capture block. v5 may have changed; "
            "search v5 for `top_camera` and adjust this regex in make_v6_from_v5.py."
        )
    replacement = (
        "        # >>> multicam: record obs_t/images_t paired with action_t before stepping\n"
        "        robot_state = robot.data.joint_pos[0]\n"
        "        cam_records = mc.capture_all_cams(env, cam_names)\n"
        "        saver.record_step(robot_state, final_action[0], cam_records)\n"
        "\n"
        "        # Step action_t to advance the simulation\n"
        "        obs, rewards, dones, _, _ = env.step(final_action)\n"
    )
    return pat.sub(lambda _m: replacement, s, count=1)


# ----------------------------------------------------------------------------
# Edit 7: replace the trailing quick-run note copied from v5 with v6 commands.
# ----------------------------------------------------------------------------
def _replace_footer_usage(s: str) -> str:
    pat = re.compile(r'\n"""\n\(isaac_lerobot\)[\s\S]*?\n"""[ \t]*$', re.MULTILINE)
    if not pat.search(s):
        return s
    replacement = '''\n"""
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v6.py /Developer/IsaacLab/
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_multicam_addons.py /Developer/IsaacLab/

./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --list_cams \\
    --env lift-ik-rel --cams top,left,right,front,wrist --cam_hw 64,64

./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --autorun \\
    --env lift-ik-rel --max_demos 50 \\
    --cams top,left,right,front,wrist --cam_hw 480,640 \\
    --save_dir logs/demos_multicam_lift

./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --autorun \\
    --env stack-ik-rel --max_demos 50 \\
    --cams top,left,right,front,wrist --cam_hw 480,640 \\
    --save_dir logs/demos_multicam_stack
"""'''
    return pat.sub(replacement, s, count=1)


# ----------------------------------------------------------------------------
# Apply all edits in order
# ----------------------------------------------------------------------------
EDITS = [
    ("docstring", _replace_top_docstring),
    ("addons import", _add_addons_import),
    ("CLI flags", _add_cli_flags),
    ("scene cfg call", _inject_scene_cfg_call),
    ("disable Fabric", _disable_fabric_for_multicam),
    ("saver init", _replace_saver_init),
    ("capture block", _replace_capture_block),
    ("footer usage", _replace_footer_usage),
]


out = src
for name, fn in EDITS:
    try:
        out = fn(out)
        print(f"  applied: {name}")
    except Exception as e:
        raise SystemExit(f"FAILED at edit '{name}': {e}\n"
                         f"v5 may have diverged from what this generator expects. "
                         f"Inspect the relevant regex in {Path(__file__).name}.")

V6.write_text(out)
print(f"\nwrote {V6}  ({len(out)} chars)")
print("Next: copy isaac_auto_collector_v6.py + isaac_multicam_addons.py to "
      "your IsaacLab working dir, then run with --cams top,left,right,front,wrist.")
