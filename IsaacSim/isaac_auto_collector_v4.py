"""
Isaac Sim Automatic Data Collection V4
=======================================

Improvements over V3:
  1. Fixed: Gripper no longer slides forward before lifting
            → XY locked during GRASP via grasp_lock_xy
  2. Fixed: No strange post-pick arm gestures
            → LIFT uses a fixed world target set at GRASP entry
  3. Fixed: VERIFY holds lifted pose (no zero-action drift)
  4. Fixed: Environment randomization actually works
            → Walks USD shader tree to set inputs:diffuse_color_constant on OmniPBR;
               handles instanceable USD prims (dex_cube_instanceable.usd) by writing
               to the prototype; falls back to binding a new UsdPreviewSurface material
  5. New:   --env selects from a registry of IK-Rel manipulation environments,
            each with individually tuned controller parameters for best success rate
  6. New:   Optional basket drop task (--use_basket)
  7. New:   Failed demo saving (--save_failed)

Supported environments (--env):
    lift-ik-rel   Isaac-Lift-Cube-Franka-IK-Rel-v0       [DEFAULT, best tested]
    stack-ik-rel  Isaac-Stack-Cube-Franka-IK-Rel-v0      [pick cube_1, stack on cube_2]
    reach-ik-rel  Isaac-Reach-Franka-IK-Rel-v0           [reach only, no pick]
    open-drawer   Isaac-Open-Drawer-Franka-IK-Rel-v0     [drawer pull, needs tuning]

    All use IK-Rel (7-D Cartesian delta) action space — the same as our controller.

    NOT supported (different action spaces, incompatible with IK-Rel controller):
        Isaac-Lift-Cube-Franka-IK-Abs-v0  — expects absolute EE pose commands
        Isaac-Lift-Cube-Franka-v0         — expects joint-angle commands

Run:
    conda activate isaac_lerobot
    cd /Developer/IsaacLab
    cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v4.py .
    ./isaaclab.sh -p isaac_auto_collector_v4.py --enable_cameras [--autorun] [options]

Options:
    --env NAME          Select environment preset (default: lift-ik-rel)
    --list_envs         Print available environments and exit
    --autorun           Start CONTINUOUS collection immediately
    --max_demos N       Target successful demos (default 50)
    --save_dir PATH     Output directory (default logs/demos)
    --use_basket        Pick + drop into basket task
    --no_randomize_env  Disable per-episode visual randomization
    --save_failed       Save failed episodes to save_dir/failed/

On failed demos for LeRobot (ACT, pi0, GR00T):
    - ACT / pi0 / GR00T: SUCCESS only. Failed demos teach wrong actions.
    - Offline RL / IQL: can use failures → --save_failed
    - Recovery demos: already captured in success episodes (retry stumbles included).
"""

import argparse
import socket
import torch
import h5py
import numpy as np
import time
import os
import json
import sys
from dataclasses import dataclass
from typing import Optional
from datetime import datetime
import zmq

from isaaclab.app import AppLauncher


# ============================================================================
#  Environment Preset Registry
# ============================================================================
@dataclass
class EnvPreset:
    """
    Per-environment configuration for the pick-and-place controller.

    All heights are specified as TCP position offsets relative to the
    object centre (positive = above), except lift_height which is a
    world-frame absolute Z value.
    """
    task_id:     str          # IsaacLab registered task string
    description: str

    # Scene entity keys
    pick_key:     str          # env.scene[pick_key] = object to grasp
    place_key:    Optional[str]  # env.scene[place_key] = stack target (None = lift only)
    robot_key:    str = "robot"
    ee_body:      str = "panda_hand"
    finger_regex: str = "panda_finger_.*"

    # Controller heights (TCP relative to object centre, metres)
    approach_height:      float = 0.15
    hover_height:         float = 0.10
    grasp_height:         float = 0.0    # 0 = grasp at cube centre
    lift_height:          float = 0.28   # world-Z of lifted TCP above table

    # Place-on params (used when place_key is set, i.e. stack task)
    place_approach_height: float = 0.20  # transit height above place target
    place_release_height:  float = 0.055 # release height above place target top

    # Controller dynamics
    kp_far:              float = 5.0
    kp_near:             float = 2.5
    threshold_far:       float = 0.10
    threshold_near:      float = 0.03
    max_action:          float = 0.8
    descend_max_action:  float = 0.25
    xy_align_threshold:  float = 0.015
    z_align_threshold:   float = 0.03
    yaw_align_threshold: float = 0.08

    # Timing (frames, ~50 Hz → 1 frame = 0.02 s)
    max_approach_frames:  int = 300
    hover_settle_frames:  int = 15
    max_descend_frames:   int = 150
    grasp_settle_frames:  int = 65    # grip close duration
    max_lift_frames:      int = 150
    verify_hold_frames:   int = 25
    max_transit_frames:   int = 250   # basket / stack transit

    # Verification
    min_cube_lift_z:  float = 0.06   # world-Z of cube to confirm grasp
    max_retries:      int = 5

    # Action space — all listed envs use IK-Rel (7-D Cartesian delta + gripper).
    # The main loop auto-detects env.action_manager.total_action_dim at startup
    # and warns (but does not crash) if it differs.
    action_dim: int = 7


SUPPORTED_ENVS: dict[str, EnvPreset] = {

    # ── Primary: Franka cube lift with relative IK ───────────────────────────
    "lift-ik-rel": EnvPreset(
        task_id="Isaac-Lift-Cube-Franka-IK-Rel-v0",
        description="Franka: pick & lift cube [IK-Rel] — recommended, best tested",
        pick_key="object",
        place_key=None,
        approach_height=0.15,
        hover_height=0.10,
        grasp_height=0.0,
        lift_height=0.28,
        grasp_settle_frames=65,
        min_cube_lift_z=0.06,
        descend_max_action=0.25,
        kp_far=5.0, kp_near=2.5,
        max_retries=5,
    ),

    # ── Franka cube stack (pick cube_1, place on cube_2) ─────────────────────
    "stack-ik-rel": EnvPreset(
        task_id="Isaac-Stack-Cube-Franka-IK-Rel-v0",
        description="Franka: pick cube_1, stack on cube_2 [IK-Rel]",
        pick_key="cube_1",
        place_key="cube_2",
        approach_height=0.15,
        hover_height=0.10,
        grasp_height=0.0,
        lift_height=0.22,          # don't need to lift as high for stacking
        place_approach_height=0.18,
        place_release_height=0.06, # place just above cube_2 top (~4cm cube)
        grasp_settle_frames=70,
        min_cube_lift_z=0.06,
        descend_max_action=0.20,   # slower for precision
        kp_far=4.5, kp_near=2.0,
        xy_align_threshold=0.012,  # tighter XY for stacking
        max_retries=4,
    ),

    # ── Franka reach (no pick — just reaching to target pose) ────────────────
    "reach-ik-rel": EnvPreset(
        task_id="Isaac-Reach-Franka-IK-Rel-v0",
        description="Franka: reach to target [IK-Rel] — reach only, no pick/lift",
        pick_key="object",  # 'object' = goal marker in reach env
        place_key=None,
        approach_height=0.0,
        hover_height=0.0,
        grasp_height=-0.05,  # reach to / slightly below marker
        lift_height=0.10,
        grasp_settle_frames=30,
        min_cube_lift_z=-0.01,  # low threshold; goal is reaching not lifting
        descend_max_action=0.30,
        kp_far=6.0, kp_near=3.0,
        max_retries=3,
    ),

    # ── Franka open drawer ────────────────────────────────────────────────────
    # NOTE: The pick target is the drawer handle ('cabinet' scene entity).
    # Grasp height and XY offsets differ significantly from cube pick.
    # Auto-success rate with the generic controller will be low; treat as
    # a starting point and tune heights for your specific cabinet asset.
    "open-drawer": EnvPreset(
        task_id="Isaac-Open-Drawer-Franka-IK-Rel-v0",
        description="Franka: open cabinet drawer [IK-Rel] — needs tuning",
        pick_key="cabinet",
        place_key=None,
        approach_height=0.05,
        hover_height=0.02,
        grasp_height=-0.02,
        lift_height=0.0,     # don't lift; pull backward instead
        grasp_settle_frames=50,
        min_cube_lift_z=-0.05,
        descend_max_action=0.20,
        kp_far=4.0, kp_near=2.0,
        max_retries=2,
    ),
}


# ============================================================================
#  Argument Parsing
# ============================================================================
parser = argparse.ArgumentParser(description="Isaac Auto Collector V4")
parser.add_argument("--env", type=str, default="lift-ik-rel",
                    choices=list(SUPPORTED_ENVS.keys()),
                    help="Environment preset (see --list_envs)")
parser.add_argument("--list_envs", action="store_true",
                    help="Print available environments and exit")
parser.add_argument("--max_demos", type=int, default=50)
parser.add_argument("--save_dir", type=str, default="logs/demos")
parser.add_argument("--autorun", action="store_true", default=False,
                    help="Auto-start CONTINUOUS collection immediately")
parser.add_argument("--use_basket", action="store_true", default=False,
                    help="Add a basket target; robot must drop cube in it")
parser.add_argument("--randomize_env", action="store_true", default=True,
                    help="Randomize cube/table appearance each episode (default on)")
parser.add_argument("--no_randomize_env", dest="randomize_env", action="store_false")
parser.add_argument("--save_failed", action="store_true", default=False,
                    help="Save failed episodes to save_dir/failed/")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Handle --list_envs before launching sim
if args_cli.list_envs:
    print("\nAvailable environments:")
    for name, preset in SUPPORTED_ENVS.items():
        print(f"  {name:<20} {preset.task_id}")
        print(f"  {'':20} {preset.description}")
    print()
    sys.exit(0)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


# ============================================================================
#  Math Utilities
# ============================================================================
def get_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def get_roll_pitch(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = torch.asin(torch.clamp(sinp, -1.0, 1.0))
    return roll, pitch


def quat_rotate_point(q, point):
    w, x, y, z = q[0], q[1], q[2], q[3]
    px, py, pz = point[0], point[1], point[2]
    rx = px + 2.0 * (-(y*y + z*z)*px + (x*y - w*z)*py + (x*z + w*y)*pz)
    ry = py + 2.0 * ((x*y + w*z)*px - (x*x + z*z)*py + (y*z - w*x)*pz)
    rz = pz + 2.0 * ((x*z - w*y)*px + (y*z + w*x)*py - (x*x + y*y)*pz)
    return torch.stack([rx, ry, rz])


# ============================================================================
#  UDP Receiver
# ============================================================================
class UDPTeleopReceiver:
    def __init__(self, port=8212, device="cuda:0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.device = device
        self.current_action = torch.zeros((1, 7), device=self.device)

    def get_action(self):
        latest_data = None
        try:
            while True:
                data, _ = self.sock.recvfrom(1024)
                latest_data = data
        except BlockingIOError:
            pass
        if latest_data:
            try:
                vals = [float(x) for x in latest_data.decode().split(',')]
                if len(vals) == 7:
                    self.current_action = torch.tensor([vals], device=self.device)
            except Exception as e:
                print(f"  UDP parse error: {e}")
        return self.current_action


# ============================================================================
#  Environment Randomizer  (corrected — uses actual USD shader attributes)
# ============================================================================
class EnvRandomizer:
    """
    Randomizes cube and table visual appearance by modifying USD shader inputs.

    WHY displayColor DOES NOT WORK:
        Isaac Sim renders objects using OmniPBR materials (an MDL shader).
        displayColor is a rendering hint that is completely overridden by any
        bound material. Setting it has no visible effect.

    CORRECT APPROACH:
        1. Walk the USD prim tree under the asset root.
        2. For instanceable assets (e.g. dex_cube_instanceable.usd) the
           instance proxy children are read-only; get the prototype via
           prim.GetPrototype() and walk that instead.
        3. Find every Shader prim and set the colour on known inputs:
               inputs:diffuse_color_constant  ← OmniPBR
               inputs:diffuseColor            ← UsdPreviewSurface
        4. If no shader is found (bare geometry), create and bind a new
           UsdPreviewSurface material.

    CUBE SCALE:
        The xformOp:scale of the root prim is modified so both visual and
        physics shapes are scaled uniformly. Works only on non-instanceable
        prims or if PhysX allows runtime scale changes for the given shape.
    """

    CUBE_COLORS = [
        ("red",     (0.85, 0.15, 0.10)),
        ("green",   (0.10, 0.75, 0.20)),
        ("blue",    (0.10, 0.30, 0.90)),
        ("yellow",  (0.95, 0.85, 0.05)),
        ("orange",  (0.95, 0.50, 0.05)),
        ("purple",  (0.55, 0.10, 0.80)),
        ("cyan",    (0.05, 0.80, 0.85)),
        ("white",   (0.92, 0.92, 0.92)),
        ("grey",    (0.48, 0.48, 0.48)),
        ("pink",    (0.95, 0.45, 0.68)),
    ]
    TABLE_COLORS = [
        ("wood_light", (0.82, 0.65, 0.45)),
        ("wood_dark",  (0.45, 0.28, 0.12)),
        ("grey_desk",  (0.60, 0.60, 0.62)),
        ("white_desk", (0.92, 0.92, 0.92)),
        ("green_felt", (0.20, 0.55, 0.25)),
        ("blue_felt",  (0.15, 0.30, 0.60)),
    ]

    # Shader input names that hold the diffuse/albedo colour, in priority order
    COLOR_INPUTS = [
        "inputs:diffuse_color_constant",   # OmniPBR (standard in Isaac Sim)
        "inputs:diffuseColor",             # UsdPreviewSurface
        "inputs:albedoColor",              # some custom MDL shaders
        "inputs:base_color",               # generic PBR
    ]

    def __init__(self):
        self._stage = None
        self._mat_cache: dict[str, object] = {}   # prim_path → bound material

    # ── Stage access ──────────────────────────────────────────────────────────

    def _stage_get(self):
        if self._stage is None:
            try:
                import omni.usd
                self._stage = omni.usd.get_context().get_stage()
            except Exception:
                pass
        return self._stage

    # ── Core USD helpers ─────────────────────────────────────────────────────

    def _resolve_prim(self, path: str):
        """
        Return the prim at *path*.  If it is a USD instance (instanceable
        asset), return the prototype instead — the prototype is the writable
        source of truth for material/shader attributes.
        """
        stage = self._stage_get()
        if stage is None:
            return None
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        if prim.IsInstance():
            proto = prim.GetPrototype()
            return proto if proto.IsValid() else None
        return prim

    def _iter_shaders(self, root_prim):
        """Yield every Shader prim in the subtree rooted at *root_prim*."""
        try:
            from pxr import Usd
            for prim in Usd.PrimRange(root_prim):
                if prim.GetTypeName() == "Shader":
                    yield prim
        except Exception:
            pass

    def _set_shader_color(self, root_prim, rgb) -> bool:
        """
        Walk all shaders under *root_prim* and set their diffuse colour.
        Returns True if at least one attribute was modified.
        """
        try:
            from pxr import Gf
            hit = False
            for shader in self._iter_shaders(root_prim):
                for inp_name in self.COLOR_INPUTS:
                    attr = shader.GetAttribute(inp_name)
                    if attr and attr.IsValid():
                        attr.Set(Gf.Vec3f(*rgb))
                        hit = True
                        break  # one colour input per shader is enough
            return hit
        except Exception as e:
            print(f"  [Rand] _set_shader_color: {e}")
            return False

    def _bind_new_material(self, instance_prim_path: str, rgb, roughness=0.45):
        """
        Create (or reuse + update) a UsdPreviewSurface material and bind it
        to the prim at *instance_prim_path*.  Binding on the instance prim
        itself (not the prototype) gives a per-instance override that does not
        affect other instances.
        """
        try:
            from pxr import UsdShade, Sdf, Gf
            stage = self._stage_get()
            if stage is None:
                return False

            # One persistent material path per target prim (reuse across episodes)
            safe = instance_prim_path.replace("/", "_").strip("_")
            mat_path = f"/World/_RandMats/{safe}"

            mat_prim = stage.GetPrimAtPath(mat_path)
            if mat_prim.IsValid():
                # Material already exists — just update the colour
                shader_prim = stage.GetPrimAtPath(mat_path + "/PrevSurf")
                if shader_prim.IsValid():
                    for inp in self.COLOR_INPUTS:
                        attr = shader_prim.GetAttribute(inp)
                        if attr and attr.IsValid():
                            attr.Set(Gf.Vec3f(*rgb))
                            break
            else:
                # First call: define a new UsdPreviewSurface material
                mat = UsdShade.Material.Define(stage, mat_path)
                shader = UsdShade.Shader.Define(stage, mat_path + "/PrevSurf")
                shader.CreateIdAttr("UsdPreviewSurface")
                shader.CreateInput("diffuseColor",
                                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
                shader.CreateInput("roughness",
                                   Sdf.ValueTypeNames.Float).Set(roughness)
                shader.CreateInput("metallic",
                                   Sdf.ValueTypeNames.Float).Set(0.0)
                mat.CreateSurfaceOutput().ConnectToSource(
                    shader.ConnectableAPI(), "surface")

            # Bind to the instance prim (strong binding overrides prototype mat)
            tgt = stage.GetPrimAtPath(instance_prim_path)
            if tgt.IsValid():
                mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
                UsdShade.MaterialBindingAPI.Apply(tgt).Bind(
                    mat,
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                )
                return True
        except Exception as e:
            print(f"  [Rand] _bind_new_material: {e}")
        return False

    def _set_scale(self, prim_path: str, sx, sy, sz) -> bool:
        """Modify xformOp:scale on the prim (visual + physics scaled together)."""
        try:
            from pxr import UsdGeom, Gf
            stage = self._stage_get()
            if stage is None:
                return False
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return False
            xf = UsdGeom.Xformable(prim)
            ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
            if "xformOp:scale" in ops:
                ops["xformOp:scale"].Set(Gf.Vec3f(sx, sy, sz))
            else:
                xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
            return True
        except Exception as e:
            print(f"  [Rand] _set_scale: {e}")
            return False

    # ── Public API ────────────────────────────────────────────────────────────

    def randomize_object(self, prim_path: str) -> dict:
        """
        Randomize the colour (and optionally scale) of the object at *prim_path*.
        Returns a metadata dict for HDF5 logging.

        Strategy:
            1. Resolve instance → prototype if needed.
            2. Walk prototype shaders and set colour there.
            3. Also bind a new material to the instance prim for an
               instance-specific override (works even for instanceable USD).
        """
        name, rgb = self.CUBE_COLORS[np.random.randint(len(self.CUBE_COLORS))]
        roughness = float(np.random.uniform(0.2, 0.75))
        meta = {"cube_color": name, "cube_rgb": list(rgb)}

        proto = self._resolve_prim(prim_path)
        if proto is not None:
            ok = self._set_shader_color(proto, rgb)
            if not ok:
                # No existing shaders found — bind a fresh material on the instance
                self._bind_new_material(prim_path, rgb, roughness)
        else:
            # Prim not found at given path — try binding directly anyway
            self._bind_new_material(prim_path, rgb, roughness)

        # Scale variation: rectangular prism (±15% per axis, ±20% base)
        base = float(np.random.uniform(0.85, 1.20))
        sx = base * float(np.random.uniform(0.85, 1.15))
        sy = base * float(np.random.uniform(0.85, 1.15))
        sz = base * float(np.random.uniform(0.90, 1.10))
        self._set_scale(prim_path, sx, sy, sz)
        meta["cube_scale"] = [round(sx, 3), round(sy, 3), round(sz, 3)]

        print(f"  [Rand] cube={name} scale=({sx:.2f},{sy:.2f},{sz:.2f}) "
              f"roughness={roughness:.2f}")
        return meta

    def randomize_table(self, prim_path: str) -> dict:
        """Randomize table colour. Returns metadata."""
        name, rgb = self.TABLE_COLORS[np.random.randint(len(self.TABLE_COLORS))]
        roughness = float(np.random.uniform(0.4, 0.85))
        meta = {"table_color": name}

        proto = self._resolve_prim(prim_path)
        if proto is not None:
            ok = self._set_shader_color(proto, rgb)
            if not ok:
                self._bind_new_material(prim_path, rgb, roughness)
        else:
            self._bind_new_material(prim_path, rgb, roughness)

        print(f"  [Rand] table={name}")
        return meta

    def randomize(self, cube_prim_path: str, table_prim_path: Optional[str] = None) -> dict:
        """
        Convenience wrapper: randomize cube (required) and table (optional).
        Pass the prim path obtained from env.scene[key].prim_paths[0].
        """
        meta = self.randomize_object(cube_prim_path)
        if table_prim_path:
            meta.update(self.randomize_table(table_prim_path))
        return meta


# ============================================================================
#  Basket Target
# ============================================================================
class BasketTarget:
    """
    Visual basket indicator placed at a random reachable table position.
    Success = cube within BASKET_RADIUS of centre AND resting on table.
    """
    BASKET_RADIUS = 0.07
    BASKET_HEIGHT = 0.04

    def __init__(self, device):
        self.device = device
        self.basket_pos: Optional[torch.Tensor] = None
        self._table_z = 0.0
        self._spawned = False
        self._prim_root = "/World/_Basket"

    def spawn_or_move(self, table_z=0.0) -> torch.Tensor:
        self._table_z = table_z
        x = float(np.random.uniform(0.38, 0.60))
        y = float(np.random.uniform(-0.22, 0.22))
        z = table_z + 0.001
        self.basket_pos = torch.tensor([x, y, z], device=self.device)
        if not self._spawned:
            self._spawn_visual(x, y, z)
        else:
            self._move_visual(x, y, z)
        print(f"  [Basket] Target ({x:.3f}, {y:.3f}, {z:.3f})")
        return self.basket_pos

    def _spawn_visual(self, x, y, z):
        try:
            import omni.usd
            from pxr import UsdGeom, UsdShade, Sdf, Gf, Vt
            stage = omni.usd.get_context().get_stage()
            cyl_path = self._prim_root + "/Ring"
            cyl = UsdGeom.Cylinder.Define(stage, cyl_path)
            cyl.GetRadiusAttr().Set(self.BASKET_RADIUS)
            cyl.GetHeightAttr().Set(self.BASKET_HEIGHT)
            cyl.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.4, 0.0)]))
            xf = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(cyl_path))
            xf.SetTranslate(Gf.Vec3d(x, y, z + self.BASKET_HEIGHT / 2))
            self._spawned = True
        except Exception as e:
            print(f"  [Basket] spawn visual: {e}")

    def _move_visual(self, x, y, z):
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            stage = omni.usd.get_context().get_stage()
            xf = UsdGeom.XformCommonAPI(
                stage.GetPrimAtPath(self._prim_root + "/Ring"))
            xf.SetTranslate(Gf.Vec3d(x, y, z + self.BASKET_HEIGHT / 2))
        except Exception as e:
            print(f"  [Basket] move visual: {e}")

    def check_drop_success(self, cube_pos: torch.Tensor) -> bool:
        if self.basket_pos is None:
            return False
        dx = (cube_pos[0] - self.basket_pos[0]).item()
        dy = (cube_pos[1] - self.basket_pos[1]).item()
        return (dx**2 + dy**2)**0.5 < self.BASKET_RADIUS and \
               cube_pos[2].item() < self._table_z + 0.08

    def get_pos(self) -> Optional[torch.Tensor]:
        return self.basket_pos


# ============================================================================
#  Pick & Place Controller V4
# ============================================================================
class PickAndPlaceController:
    """
    State-machine pick-and-place controller driven by an EnvPreset config.

    Phases (lift-only):
        APPROACH_XY → HOVER → OPEN_GRIP → DESCEND → GRASP → LIFT
        → VERIFY_LIFT → DONE_SUCCESS | DONE_FAIL

    Additional phases (stack or basket task):
        VERIFY_LIFT → MOVE_TO_TARGET → DESCEND_TO_TARGET
        → RELEASE → VERIFY_PLACE → DONE_SUCCESS | DONE_FAIL

    Key design fixes vs V3:
        GRASP:       lock_xy=True — XY commands zeroed, prevents sliding
        LIFT:        lift_target_world is fixed when entering LIFT, no chasing
        VERIFY_LIFT: actively holds lifted pose (not zero action = no drift)
    """

    APPROACH_XY      = "APPROACH_XY"
    HOVER            = "HOVER"
    OPEN_GRIP        = "OPEN_GRIP"
    DESCEND          = "DESCEND"
    GRASP            = "GRASP"
    LIFT             = "LIFT"
    VERIFY_LIFT      = "VERIFY_LIFT"
    MOVE_TO_TARGET   = "MOVE_TO_TARGET"
    DESCEND_TO_TARGET = "DESCEND_TO_TARGET"
    RELEASE          = "RELEASE"
    VERIFY_PLACE     = "VERIFY_PLACE"
    DONE_SUCCESS     = "DONE_SUCCESS"
    DONE_FAIL        = "DONE_FAIL"

    # Franka panda_hand → TCP body offset in local frame
    IK_BODY_OFFSET = torch.tensor([0.0, 0.0, 0.107])

    def __init__(self, preset: EnvPreset, device: str,
                 use_basket=False, use_place=False):
        self.p = preset
        self.device = device
        self.use_basket = use_basket
        self.use_place  = use_place   # True when place_key is set (stack task)
        self.body_offset = self.IK_BODY_OFFSET.to(device)

        self.phase        = self.APPROACH_XY
        self.phase_timer  = 0
        self.retry_count  = 0
        self.debug_counter = 0

        # Fixed targets set at transitions (prevent drifting / chasing)
        self.lift_target_world: Optional[torch.Tensor] = None
        self.grasp_lock_xy:     Optional[torch.Tensor] = None
        self.obj_lock_pos:      Optional[torch.Tensor] = None  # cube XY locked at OPEN_GRIP exit
        self.place_target:      Optional[torch.Tensor] = None  # basket or stack top

        self._prev_action = torch.zeros(7, device=device)

    def set_place_target(self, target: torch.Tensor):
        """Set basket position or stack target (called externally per episode)."""
        self.place_target = target

    def reset(self):
        self.phase        = self.APPROACH_XY
        self.phase_timer  = 0
        self.retry_count  = 0
        self.debug_counter = 0
        self.lift_target_world = None
        self.grasp_lock_xy     = None
        self.obj_lock_pos      = None
        self._prev_action = torch.zeros(7, device=self.device)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _adaptive_gain(self, dist):
        p = self.p
        if dist > p.threshold_far:
            return p.kp_far
        elif dist < p.threshold_near:
            return p.kp_near
        t = (dist - p.threshold_near) / (p.threshold_far - p.threshold_near)
        return p.kp_near + t * (p.kp_far - p.kp_near)

    def _tcp(self, ee_pos, ee_quat) -> torch.Tensor:
        return ee_pos + quat_rotate_point(ee_quat, self.body_offset)

    def _action(self, tcp_target, target_yaw, ee_pos, ee_quat,
                grip=1.0, max_act=None, lock_xy=False):
        """
        Compute a 7-D IK-Rel action toward tcp_target.

        lock_xy=True: zero out X/Y commands (used during GRASP to prevent
                      the gripper from chasing a sliding cube).
        """
        if max_act is None:
            max_act = self.p.max_action
        act = torch.zeros(7, device=self.device)
        tcp_pos = self._tcp(ee_pos, ee_quat)
        delta   = tcp_target - tcp_pos
        dist_xy = torch.norm(delta[:2])
        dist_z  = torch.abs(delta[2])
        kp      = self._adaptive_gain(torch.norm(delta).item())
        cmd     = delta * kp

        if lock_xy:
            act[0] = act[1] = 0.0
        else:
            act[0] = torch.clamp(cmd[0], -max_act, max_act)
            act[1] = torch.clamp(cmd[1], -max_act, max_act)
        act[2] = torch.clamp(cmd[2], -max_act, max_act)

        # Keep gripper pointing straight down
        roll, pitch = get_roll_pitch(ee_quat)
        des_roll = torch.tensor(3.14159, device=self.device)
        roll_err  = torch.atan2(torch.sin(des_roll - roll), torch.cos(des_roll - roll))
        pitch_err = -pitch
        act[3] = torch.clamp(roll_err  * 1.0, -max_act * 0.5, max_act * 0.5)
        act[4] = torch.clamp(pitch_err * 1.0, -max_act * 0.5, max_act * 0.5)

        cur_yaw = get_yaw(ee_quat)
        yaw_err = torch.atan2(torch.sin(target_yaw - cur_yaw),
                              torch.cos(target_yaw - cur_yaw))
        act[5] = torch.clamp(yaw_err * 2.0, -max_act, max_act)
        act[6] = grip

        # Exponential smoothing (reduce jitter; never smooth gripper)
        smoothed = 0.7 * act + 0.3 * self._prev_action
        smoothed[6] = grip
        self._prev_action = smoothed.clone()
        return smoothed, dist_xy.item(), dist_z.item(), abs(yaw_err.item())

    def _transition(self, new_phase):
        if new_phase != self.phase:
            print(f"  → {self.phase} → {new_phase} (t={self.phase_timer})")
        self.phase = new_phase
        self.phase_timer = 0
        self._prev_action = torch.zeros(7, device=self.device)

    def _retry_or_fail(self) -> bool:
        """Increment retry counter. Returns True if giving up (DONE_FAIL)."""
        self.retry_count += 1
        if self.retry_count >= self.p.max_retries:
            self.phase = self.DONE_FAIL
            print(f"  Max retries ({self.p.max_retries}) reached.")
            return True
        print(f"  Retry {self.retry_count}/{self.p.max_retries}")
        self.lift_target_world = None
        self.grasp_lock_xy     = None
        self.obj_lock_pos      = None
        self._transition(self.APPROACH_XY)
        return False

    # ── Main step ─────────────────────────────────────────────────────────────

    def compute(self, obj_pos, obj_quat, ee_pos, ee_quat,
                place_obj_pos=None):
        """
        Run one controller step.

        Args:
            obj_pos:       world position of primary object (cube)
            obj_quat:      world quaternion of primary object
            ee_pos:        panda_hand world position
            ee_quat:       panda_hand world quaternion
            place_obj_pos: position of the place-target object (for stack task)

        Returns:
            action:      (1,7) tensor
            phase:       current phase string
            is_terminal: True when episode should be saved/reset
        """
        p = self.p
        self.phase_timer   += 1
        self.debug_counter += 1
        target_yaw = get_yaw(obj_quat)
        is_terminal = False

        if self.debug_counter % 50 == 0:
            tcp = self._tcp(ee_pos, ee_quat)
            print(f"  [DBG] {self.phase} t={self.phase_timer} "
                  f"tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) "
                  f"obj=({obj_pos[0]:.3f},{obj_pos[1]:.3f},{obj_pos[2]:.3f})")

        # ── APPROACH_XY ──────────────────────────────────────────────────────
        if self.phase == self.APPROACH_XY:
            tgt = obj_pos.clone(); tgt[2] += p.approach_height
            act, dxy, dz, dyaw = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                              grip=1.0)
            if (dxy**2 + dz**2)**0.5 < p.xy_align_threshold * 2 \
                    and dyaw < p.yaw_align_threshold:
                self._transition(self.HOVER)
            elif self.phase_timer > p.max_approach_frames:
                print(f"  APPROACH_XY timeout → HOVER")
                self._transition(self.HOVER)

        # ── HOVER ─────────────────────────────────────────────────────────────
        elif self.phase == self.HOVER:
            tgt = obj_pos.clone(); tgt[2] += p.hover_height
            act, dxy, _, dyaw = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                             grip=1.0)
            if self.phase_timer > p.hover_settle_frames:
                if dxy < p.xy_align_threshold and dyaw < p.yaw_align_threshold:
                    self._transition(self.OPEN_GRIP)
                elif self.phase_timer > p.hover_settle_frames * 4:
                    self._transition(self.OPEN_GRIP)

        # ── OPEN_GRIP ─────────────────────────────────────────────────────────
        elif self.phase == self.OPEN_GRIP:
            tgt = obj_pos.clone(); tgt[2] += p.hover_height
            act, *_ = self._action(tgt, target_yaw, ee_pos, ee_quat, grip=1.0)
            if self.phase_timer > 15:
                # Snapshot cube XY right before descending.
                # DESCEND will target this fixed XY — the gripper will not chase
                # the cube if contact pushes it sideways.
                self.obj_lock_pos = obj_pos.clone()
                self._transition(self.DESCEND)

        # ── DESCEND ──────────────────────────────────────────────────────────
        # Uses obj_lock_pos for XY (fixed at OPEN_GRIP exit) so the gripper
        # descends to a stable XY target.  Z still tracks live cube height in
        # case it settled lower than expected.
        # Cube-escape guard: if the cube slides > 7 cm from the locked XY
        # (usually because a previous grasp attempt bumped it), abort and
        # re-approach from scratch so the gripper re-centres over the cube.
        elif self.phase == self.DESCEND:
            if self.obj_lock_pos is None:
                self.obj_lock_pos = obj_pos.clone()
            tgt = self.obj_lock_pos.clone()
            tgt[2] = obj_pos[2].item() + p.grasp_height  # live Z (let cube settle)

            act, dxy, dz, _ = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                           grip=1.0,
                                           max_act=p.descend_max_action)

            # How far has the cube drifted from where we planned to descend?
            cube_escape = torch.norm(obj_pos[:2] - self.obj_lock_pos[:2]).item()

            if self.phase_timer % 20 == 0:
                print(f"  [DESCEND] dxy={dxy:.3f} dz={dz:.3f} "
                      f"escape={cube_escape:.3f}")

            # Abort if cube has slid away (being pushed by previous contact)
            if cube_escape > 0.07 and self.phase_timer > 25:
                print(f"  DESCEND: cube escaped {cube_escape:.3f}m — re-approach")
                self.obj_lock_pos = None
                if self._retry_or_fail():
                    return act.unsqueeze(0), self.phase, True

            elif (dxy**2 + dz**2)**0.5 < p.z_align_threshold:
                self.grasp_lock_xy = self._tcp(ee_pos, ee_quat)[:2].clone()
                self._transition(self.GRASP)
            elif self.phase_timer > p.max_descend_frames:
                self.grasp_lock_xy = self._tcp(ee_pos, ee_quat)[:2].clone()
                print(f"  DESCEND timeout → GRASP")
                self._transition(self.GRASP)

        # ── GRASP ─────────────────────────────────────────────────────────────
        # XY is LOCKED at grasp_lock_xy: prevents forward sliding on contact.
        elif self.phase == self.GRASP:
            tgt = obj_pos.clone(); tgt[2] += p.grasp_height
            if self.grasp_lock_xy is not None:
                tgt[0] = self.grasp_lock_xy[0]
                tgt[1] = self.grasp_lock_xy[1]
            act, *_ = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                   grip=-1.0,
                                   max_act=p.descend_max_action,
                                   lock_xy=True)
            act[2] = torch.clamp(act[2], -0.08, 0.08)   # gentle Z only
            if self.phase_timer > p.grasp_settle_frames:
                # Capture fixed lift target now (object still at table height)
                self.lift_target_world = obj_pos.clone()
                self.lift_target_world[2] += p.lift_height
                self._transition(self.LIFT)

        # ── LIFT ──────────────────────────────────────────────────────────────
        # Uses a FIXED world target (set at GRASP → LIFT transition).
        # No chasing of the moving lifted object.
        elif self.phase == self.LIFT:
            if self.lift_target_world is None:
                self.lift_target_world = obj_pos.clone()
                self.lift_target_world[2] += p.lift_height
            act, _, dz, _ = self._action(self.lift_target_world, target_yaw,
                                         ee_pos, ee_quat, grip=-1.0)
            if dz < 0.03 or self.phase_timer > p.max_lift_frames:
                self._transition(self.VERIFY_LIFT)

        # ── VERIFY_LIFT ───────────────────────────────────────────────────────
        # Actively hold lifted pose (not zero action → no arm drift).
        elif self.phase == self.VERIFY_LIFT:
            if self.lift_target_world is not None:
                tgt = self.lift_target_world
            else:
                tgt = obj_pos.clone()
                tgt[2] += p.lift_height
            act, *_ = self._action(tgt, target_yaw, ee_pos, ee_quat, grip=-1.0)

            if self.phase_timer > p.verify_hold_frames:
                cube_z = obj_pos[2].item()
                print(f"  [VERIFY_LIFT] cube_z={cube_z:.3f} "
                      f"threshold={p.min_cube_lift_z:.3f}")
                if cube_z > p.min_cube_lift_z:
                    print(f"  GRASP OK!")
                    has_place = (self.use_basket or self.use_place) \
                                and self.place_target is not None
                    if has_place:
                        self._transition(self.MOVE_TO_TARGET)
                    else:
                        self.phase = self.DONE_SUCCESS
                        is_terminal = True
                else:
                    is_terminal = self._retry_or_fail()

        # ── MOVE_TO_TARGET (basket / stack transit) ───────────────────────────
        elif self.phase == self.MOVE_TO_TARGET:
            tgt = self.place_target.clone()
            tgt[2] += self.p.place_approach_height
            act, dxy, dz, _ = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                           grip=-1.0)
            if (dxy**2 + dz**2)**0.5 < 0.03 or \
                    self.phase_timer > p.max_transit_frames:
                self._transition(self.DESCEND_TO_TARGET)

        # ── DESCEND_TO_TARGET ─────────────────────────────────────────────────
        elif self.phase == self.DESCEND_TO_TARGET:
            tgt = self.place_target.clone()
            tgt[2] += self.p.place_release_height
            act, _, dz, _ = self._action(tgt, target_yaw, ee_pos, ee_quat,
                                         grip=-1.0,
                                         max_act=p.descend_max_action)
            if dz < 0.02 or self.phase_timer > 100:
                self._transition(self.RELEASE)

        # ── RELEASE ───────────────────────────────────────────────────────────
        elif self.phase == self.RELEASE:
            tgt = self.place_target.clone()
            tgt[2] += self.p.place_release_height
            act, *_ = self._action(tgt, target_yaw, ee_pos, ee_quat, grip=1.0)
            if self.phase_timer > 30:
                self._transition(self.VERIFY_PLACE)

        # ── VERIFY_PLACE ──────────────────────────────────────────────────────
        elif self.phase == self.VERIFY_PLACE:
            tgt = self.place_target.clone()
            tgt[2] += self.p.place_approach_height
            act, *_ = self._action(tgt, target_yaw, ee_pos, ee_quat, grip=1.0)
            if self.phase_timer > 20:
                is_terminal = True
                dx = (obj_pos[0] - self.place_target[0]).item()
                dy = (obj_pos[1] - self.place_target[1]).item()
                dist = (dx**2 + dy**2)**0.5
                cube_z = obj_pos[2].item()
                print(f"  [VERIFY_PLACE] dist={dist:.3f} cube_z={cube_z:.3f}")
                if dist < 0.10 and cube_z < 0.12:
                    self.phase = self.DONE_SUCCESS
                    print(f"  PLACE VERIFIED!")
                else:
                    is_terminal = self._retry_or_fail()
                    if not self.is_failed:
                        is_terminal = False

        else:
            act = torch.zeros(7, device=self.device)

        return act.unsqueeze(0), self.phase, is_terminal

    @property
    def is_success(self): return self.phase == self.DONE_SUCCESS
    @property
    def is_failed(self):  return self.phase == self.DONE_FAIL


# ============================================================================
#  LeRobot-Compatible Data Saver
# ============================================================================
class LeRobotDemoSaver:
    def __init__(self, save_dir="logs/demos", save_failed=False, task_id=""):
        self.save_dir    = save_dir
        self.save_failed = save_failed
        self.task_id     = task_id
        os.makedirs(save_dir, exist_ok=True)
        if save_failed:
            os.makedirs(os.path.join(save_dir, "failed"), exist_ok=True)

        self.states  = []
        self.actions = []
        self.images  = []
        self.demo_count    = 0
        self.success_count = 0
        self.fail_count    = 0

    def record_step(self, joint_pos, action, image=None):
        self.states.append(
            joint_pos.cpu().numpy() if torch.is_tensor(joint_pos) else joint_pos)
        self.actions.append(
            action.cpu().numpy() if torch.is_tensor(action) else action)
        if image is not None:
            self.images.append(
                image.cpu().numpy() if torch.is_tensor(image) else image)

    def save(self, success=True, episode_info=None):
        if len(self.actions) < 10:
            print(f"  Trajectory too short ({len(self.actions)} steps), skipping.")
            self.clear(); return None

        if not success:
            self.fail_count += 1
            if not self.save_failed:
                print(f"  Discarding failed episode (fail #{self.fail_count})")
                self.clear(); return None
            subdir = os.path.join(self.save_dir, "failed")
            idx = self.fail_count
        else:
            subdir = self.save_dir
            idx = self.demo_count

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = "demo" if success else "fail"
        filename = os.path.join(subdir, f"{tag}_{idx:04d}_{timestamp}.hdf5")

        with h5py.File(filename, "w") as f:
            f.create_dataset("obs/state",   data=np.array(self.states),
                             compression="gzip", compression_opts=4)
            f.create_dataset("actions",     data=np.array(self.actions),
                             compression="gzip", compression_opts=4)
            if self.images:
                f.create_dataset("obs/images/top", data=np.array(self.images),
                                 compression="gzip", compression_opts=4)
            meta = {
                "timestamp": timestamp,
                "episode_index": idx,
                "num_frames": len(self.actions),
                "success": success,
                "format_version": "lerobot_v1",
                "action_space": "ik_rel_7d",
                "robot": "franka_panda",
                "task": self.task_id,
            }
            if episode_info:
                meta.update(episode_info)
            f.attrs["metadata"] = json.dumps(meta)

        if success:
            self.demo_count    += 1
            self.success_count += 1

        print(f"  Saved: {filename} ({len(self.actions)} steps)")
        print(f"  Stats: {self.success_count} ok, {self.fail_count} fail, "
              f"{self.success_count + self.fail_count} total")
        self.clear()
        return filename

    def clear(self):
        self.states.clear(); self.actions.clear(); self.images.clear()

    @property
    def total_demos(self): return self.demo_count


# ============================================================================
#  Main Loop
# ============================================================================
def main():
    preset = SUPPORTED_ENVS[args_cli.env]
    print(f"\n  Environment: {preset.task_id}")
    print(f"  Preset:      {args_cli.env} — {preset.description}\n")

    env_cfg = parse_env_cfg(preset.task_id)
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = 3600.0

    env = ManagerBasedRLEnv(cfg=env_cfg)
    robot    = env.scene[preset.robot_key]
    ee_idx   = robot.find_bodies(preset.ee_body)[0][0]
    fj_ids   = robot.find_joints(preset.finger_regex)[0]
    lf_idx   = robot.find_bodies("panda_leftfinger")[0][0]
    rf_idx   = robot.find_bodies("panda_rightfinger")[0][0]
    print(f"  EE body: {preset.ee_body} idx={ee_idx}")
    print(f"  Finger joint ids: {fj_ids}")

    # Auto-detect the env's actual action dimension.
    # Our controller always outputs 7-D (IK-Rel format).
    # For envs that expect 8-D (IK-Abs, joint-pos), we zero-pad at step time.
    env_action_dim = env.action_manager.total_action_dim
    if env_action_dim != preset.action_dim:
        print(f"  WARNING: preset declares action_dim={preset.action_dim} "
              f"but env reports {env_action_dim}. Using {env_action_dim}.")
    ctrl_action_dim = env_action_dim   # the size we must send to env.step()
    print(f"  Action dim: {ctrl_action_dim}")

    # Determine if we have a place target (stack task or basket)
    use_place  = (preset.place_key is not None)
    use_basket = args_cli.use_basket

    controller = PickAndPlaceController(
        preset=preset, device=env.device,
        use_basket=use_basket, use_place=use_place)
    saver = LeRobotDemoSaver(
        save_dir=args_cli.save_dir,
        save_failed=args_cli.save_failed,
        task_id=preset.task_id)
    randomizer = EnvRandomizer() if args_cli.randomize_env else None
    basket     = BasketTarget(device=env.device) if use_basket else None
    max_demos  = args_cli.max_demos

    udp = UDPTeleopReceiver(device=env.device)
    zmq_ctx  = zmq.Context()
    cmd_sock = zmq_ctx.socket(zmq.REP)
    cmd_sock.bind("tcp://0.0.0.0:8213")

    print("=" * 60)
    print(f"  Isaac Auto Collector V4  [{args_cli.env}]")
    print("=" * 60)
    print(f"  Task:       {preset.task_id}")
    print(f"  Save dir:   {args_cli.save_dir}")
    print(f"  Max demos:  {max_demos}")
    print(f"  Basket:     {use_basket}   Stack: {use_place}")
    print(f"  Randomize:  {args_cli.randomize_env}")
    print(f"  SaveFailed: {args_cli.save_failed}")
    print("=" * 60)

    obs, _ = env.reset()

    # ── Helper: get prim paths for randomizer ─────────────────────────────────
    def _prim_path(key: str) -> Optional[str]:
        """
        Return the concrete USD prim path for env_0 of scene entity 'key'.

        Uses cfg.prim_path (always set, not physics-state-dependent) with the
        IsaacLab template token {ENV_REGEX_NS} resolved to /World/envs/env_0.
        Falls back to prim_paths[0] if cfg attribute is unavailable.
        """
        # Method 1: config template (reliable across resets)
        try:
            template = env.scene[key].cfg.prim_path
            path = template.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
            path = path.replace(".*", "0")   # resolve any regex wildcards
            return path
        except AttributeError:
            pass
        # Method 2: physics-view prim_paths (may fail after many resets)
        try:
            paths = env.scene[key].prim_paths
            return paths[0] if paths else None
        except Exception:
            return None

    def _table_prim_path() -> Optional[str]:
        """Try to find the table prim path."""
        stage_ctx = None
        try:
            import omni.usd
            stage_ctx = omni.usd.get_context().get_stage()
        except Exception:
            return None
        for candidate in ["Table", "table", "DiningTable", "SeattleLabTable"]:
            for env_root in ["/World/envs/env_0", "/World"]:
                p = f"{env_root}/{candidate}"
                if stage_ctx and stage_ctx.GetPrimAtPath(p).IsValid():
                    return p
        return None

    episode_meta: dict = {}
    TCP_OFFSET_Z = 0.115   # for SEMI_HOVER manual mode

    # ── Episode initialiser ───────────────────────────────────────────────────
    def start_new_episode():
        nonlocal episode_meta
        episode_meta = {}
        env.reset()
        saver.clear()
        controller.reset()
        time.sleep(0.05)  # let physics settle before USD edits

        if randomizer is not None:
            cube_path  = _prim_path(preset.pick_key)
            table_path = _table_prim_path()
            if cube_path:
                episode_meta.update(randomizer.randomize(cube_path, table_path))
            else:
                print(f"  [Rand] Could not resolve prim path for key '{preset.pick_key}'")

        if basket is not None:
            cube_pos_now = env.scene[preset.pick_key].data.root_pos_w[0]
            table_z = cube_pos_now[2].item() - 0.025
            bpos = basket.spawn_or_move(table_z=table_z)
            controller.set_place_target(bpos)
            episode_meta["basket_pos"] = bpos.tolist()
        elif use_place and preset.place_key:
            # Stack task: place target = top surface of cube_2
            # Updated each episode in the main loop (cube_2 pos may vary)
            pass   # updated below in main loop

    if args_cli.autorun:
        state = "CONTINUOUS"
        controller.reset()
        print(f"\n  AUTORUN → CONTINUOUS ({max_demos} demos)")
    else:
        state = "MANUAL"

    # ── Main simulation loop ──────────────────────────────────────────────────
    while simulation_app.is_running():
        manual_action = udp.get_action().clone()
        final_action  = manual_action.clone()

        # ZMQ commands
        try:
            cmd = cmd_sock.recv_string(flags=zmq.NOBLOCK).strip().upper()
            if   cmd == "SEMI_HOVER":
                state = "SEMI_HOVER"
                cmd_sock.send_string("SEMI_HOVER")
            elif cmd == "SEMI_GRASP":
                state = "SEMI_GRASP"
                controller.reset()
                controller.phase = PickAndPlaceController.DESCEND
                cmd_sock.send_string("SEMI_GRASP")
            elif cmd == "FULL_AUTO":
                state = "FULL_AUTO"; controller.reset()
                cmd_sock.send_string("FULL_AUTO")
            elif cmd == "CONTINUOUS":
                state = "CONTINUOUS"; controller.reset()
                cmd_sock.send_string(f"CONTINUOUS → {max_demos} demos")
            elif cmd == "SAVE":
                state = "SAVE_TRIGGERED"
                cmd_sock.send_string("SAVE")
            elif cmd == "RESET":
                state = "RESET_TRIGGERED"
                cmd_sock.send_string("RESET")
            elif cmd == "STATUS":
                cmd_sock.send_string(
                    f"state={state} demos={saver.demo_count} "
                    f"ok={saver.success_count} fail={saver.fail_count}")
            else:
                cmd_sock.send_string(f"unknown: {cmd}")
        except zmq.error.Again:
            pass

        # Human override
        if torch.sum(torch.abs(manual_action[0, :6])) > 0.01 and \
                state not in ["MANUAL", "SAVE_TRIGGERED", "RESET_TRIGGERED"]:
            print("  Human override → MANUAL")
            state = "MANUAL"

        # Scene state
        pick_pos  = env.scene[preset.pick_key].data.root_pos_w[0]
        pick_quat = env.scene[preset.pick_key].data.root_quat_w[0]
        ee_pos    = robot.data.body_pos_w[0, ee_idx]
        ee_quat   = robot.data.body_quat_w[0, ee_idx]

        # For stack task: update place target from cube_2 position
        place_obj_pos = None
        if use_place and preset.place_key:
            place_obj_pos = env.scene[preset.place_key].data.root_pos_w[0]
            # Place target = centre of cube_2 top surface (~2cm above centre)
            stack_tgt = place_obj_pos.clone()
            stack_tgt[2] += 0.025   # approximate half-height of cube_2
            controller.set_place_target(stack_tgt)

        # Finger debug
        if state in ["FULL_AUTO", "CONTINUOUS"] and controller.phase in [
            controller.GRASP, controller.LIFT, controller.VERIFY_LIFT,
            controller.MOVE_TO_TARGET,
        ]:
            if controller.phase_timer % 10 == 0:
                fp = robot.data.joint_pos[0, fj_ids]
                lf = robot.data.body_pos_w[0, lf_idx]
                rf = robot.data.body_pos_w[0, rf_idx]
                print(f"  [FINGER] joints=[{fp[0]:.4f},{fp[1]:.4f}] "
                      f"gap={abs(lf[1]-rf[1]):.4f}")

        # ── State machine ─────────────────────────────────────────────────────
        if state == "SEMI_HOVER":
            t = pick_pos.clone(); t[2] += (0.12 + TCP_OFFSET_Z)
            ty = get_yaw(pick_quat)
            delta = t - ee_pos; dist = torch.norm(delta).item()
            act = torch.zeros((1, 7), device=env.device)
            kp = 4.0 if dist > 0.05 else 2.0
            act[0, :3] = torch.clamp(delta * kp, -0.5, 0.5)
            raw_ye = torch.atan2(torch.sin(ty - get_yaw(ee_quat)),
                                 torch.cos(ty - get_yaw(ee_quat)))
            act[0, 5] = torch.clamp(raw_ye * 2.0, -0.5, 0.5)
            act[0, 6] = 1.0
            final_action = act
            if dist < 0.02:
                print("  Aligned → MANUAL"); state = "MANUAL"

        elif state == "SEMI_GRASP":
            auto_act, _, done = controller.compute(pick_pos, pick_quat,
                                                   ee_pos, ee_quat, place_obj_pos)
            final_action = auto_act
            if done:
                print("  Semi-auto done → MANUAL")
                state = "MANUAL"; controller.reset()

        elif state == "FULL_AUTO":
            auto_act, _, done = controller.compute(pick_pos, pick_quat,
                                                   ee_pos, ee_quat, place_obj_pos)
            final_action = auto_act
            if done:
                state = "SAVE_TRIGGERED" if controller.is_success \
                        else "RESET_TRIGGERED"

        elif state == "CONTINUOUS":
            if saver.total_demos >= max_demos:
                print(f"  Target {max_demos} reached! "
                      f"Rate: {saver.success_count}/"
                      f"{saver.success_count + saver.fail_count}")
                state = "MANUAL"
            else:
                auto_act, _, done = controller.compute(
                    pick_pos, pick_quat, ee_pos, ee_quat, place_obj_pos)
                final_action = auto_act
                if done:
                    ep = {"mode": "continuous", "env": args_cli.env,
                          "retry_count": controller.retry_count,
                          **episode_meta}
                    if controller.is_success:
                        saver.save(success=True,  episode_info=ep)
                    else:
                        saver.save(success=False, episode_info=ep)
                    start_new_episode()
                    time.sleep(0.1)
                    continue

        # Pad action to match env's expected dimension (e.g. 7→8 for IK-Abs).
        # The 7-D controller action covers [dx,dy,dz,droll,dpitch,dyaw,grip].
        # Extra dims are zeroed so the env receives a valid-shaped tensor.
        if final_action.shape[1] < ctrl_action_dim:
            padded = torch.zeros((1, ctrl_action_dim), device=env.device)
            padded[0, :final_action.shape[1]] = final_action[0]
            final_action = padded

        # Step
        obs, rewards, dones, _, _ = env.step(final_action)
        robot_state = robot.data.joint_pos[0]
        img = None
        if hasattr(env.scene, "sensors") and "top_camera" in env.scene.sensors:
            img = env.scene.sensors["top_camera"].data.output["rgb"][0]
        saver.record_step(robot_state, final_action[0], image=img)

        if state == "SAVE_TRIGGERED":
            saver.save(success=True, episode_info={"mode": "manual", **episode_meta})
            state = "RESET_TRIGGERED"

        if state == "RESET_TRIGGERED" or dones[0]:
            start_new_episode()
            state = "MANUAL"

    env.close()
    simulation_app.close()
    print(f"\n  Final: {saver.success_count} demos → {args_cli.save_dir}")


if __name__ == "__main__":
    main()

"""
(isaac_lerobot) lkk@rtx5090:/Developer/IsaacLab$ cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v4.py /Developer/IsaacLab/

./isaaclab.sh -p isaac_auto_collector_v4.py --enable_cameras --autorun --env lift-ik-rel   # cube lift ✅
./isaaclab.sh -p isaac_auto_collector_v4.py --enable_cameras --autorun --env stack-ik-rel  # cube stack ✅
./isaaclab.sh -p isaac_auto_collector_v4.py --enable_cameras --autorun --env reach-ik-rel  # reach ✅
./isaaclab.sh -p isaac_auto_collector_v4.py --enable_cameras --autorun --env open-drawer   # drawer (tune heights) ✅


"""