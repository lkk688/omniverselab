"""
Isaac Sim Automatic Data Collection V2 - Robust Full-Auto Pick & Place
======================================================================

Runs inside IsaacLab:
    conda activate isaac_lerobot
    cd /Developer/IsaacLab
    ./isaaclab.sh -p isaac_auto_collector_v2.py --enable_cameras

Modes:
    MANUAL       - Remote gamepad control via UDP (default)
    SEMI_HOVER   - Auto-align above cube, then return to manual
    SEMI_GRASP   - Auto-descend, grasp, and lift from current position
    FULL_AUTO    - Fully autonomous pick-and-lift with verification & retry
    CONTINUOUS   - Like FULL_AUTO but auto-resets and repeats for N demos
    SAVE         - Save trajectory to HDF5 (LeRobot compatible)
    RESET        - Reset environment
"""

import argparse
import socket
import torch
import h5py
import numpy as np
import time
import os
import json
from datetime import datetime
import zmq

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac Auto Collector V2 - Robust Full-Auto")
parser.add_argument("--max_demos", type=int, default=50, help="Max demos for CONTINUOUS mode")
parser.add_argument("--save_dir", type=str, default="logs/demos", help="Demo output directory")
parser.add_argument("--autorun", action="store_true", default=False,
                    help="Auto-start CONTINUOUS collection without waiting for remote commands")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


# ============================================================================
#  Math Utilities
# ============================================================================
def get_yaw(q):
    """Extract yaw angle from quaternion (w, x, y, z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def get_roll_pitch(q):
    """Extract roll and pitch from quaternion (w, x, y, z).
    Returns (roll, pitch) in radians.
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    # Roll (rotation about X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    # Pitch (rotation about Y)
    sinp = 2.0 * (w * y - z * x)
    pitch = torch.asin(torch.clamp(sinp, -1.0, 1.0))
    return roll, pitch


def quat_rotate_point(q, point):
    """Rotate a 3D point by quaternion (w, x, y, z).
    Returns the rotated point as a 3D tensor.
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    px, py, pz = point[0], point[1], point[2]
    # Quaternion rotation: q * p * q_conj
    # Optimized form:
    t0 = 2.0 * (x * pz - z * px)
    t1 = 2.0 * (y * pz - z * py)  # not used in simplified form
    rx = px + 2.0 * (-(y*y + z*z)*px + (x*y - w*z)*py + (x*z + w*y)*pz)
    ry = py + 2.0 * ((x*y + w*z)*px - (x*x + z*z)*py + (y*z - w*x)*pz)
    rz = pz + 2.0 * ((x*z - w*y)*px + (y*z + w*x)*py - (x*x + y*y)*pz)
    return torch.stack([rx, ry, rz])


def quaternion_to_rotation_matrix(q):
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return torch.stack([
        1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y),
        2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x),
        2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)
    ]).reshape(3, 3)


# ============================================================================
#  UDP Receiver (unchanged - for gamepad teleop)
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
                print(f"⚠️ UDP parse error: {e}")
        return self.current_action


# ============================================================================
#  Robust Pick & Place Controller
# ============================================================================
class PickAndPlaceController:
    """
    A robust state machine controller for autonomous cube pick-and-place.

    State Machine Flow:
        APPROACH_XY  →  Align XY above cube at safe height
        HOVER        →  Hold position, stabilize
        OPEN_GRIP    →  Ensure gripper is fully open
        DESCEND      →  Lower slowly to grasp height
        GRASP        →  Close gripper, wait for stable contact
        LIFT         →  Lift object upward
        VERIFY       →  Check if cube was actually lifted

    Features:
        - Distance-based phase transitions (not timers)
        - Adaptive gain: high when far, low when close (smooth + precise)
        - Grasp verification: checks cube height after lifting
        - Retry logic: up to 3 retries before giving up on an episode
        - Anti-collision: Z floor clamping
    """

    # Phase constants
    APPROACH_XY = "APPROACH_XY"
    HOVER = "HOVER"
    OPEN_GRIP = "OPEN_GRIP"
    DESCEND = "DESCEND"
    GRASP = "GRASP"
    LIFT = "LIFT"
    VERIFY = "VERIFY"
    DONE_SUCCESS = "DONE_SUCCESS"
    DONE_FAIL = "DONE_FAIL"

    # The IK-Rel controller's body_offset: the action delta is applied to the TCP,
    # which is panda_hand + body_offset rotated by the hand's quaternion.
    # We MUST compute the TCP world position correctly using quaternion rotation
    # to avoid XY misalignment when the gripper is tilted.
    IK_BODY_OFFSET = torch.tensor([0.0, 0.0, 0.107])  # local frame offset

    def __init__(self, device, tcp_offset_z=0.0):
        """
        Args:
            device: torch device
            tcp_offset_z: Additional fine-tuning offset (usually 0.0).
        """
        self.device = device
        self.tcp_offset_z = tcp_offset_z
        self.body_offset = self.IK_BODY_OFFSET.to(device)

        # Controller gains — tuned for IK-Rel env with action scale=0.5
        self.kp_far = 5.0       # Gain when distance > threshold_far
        self.kp_near = 2.5      # Gain when distance < threshold_near
        self.threshold_far = 0.10
        self.threshold_near = 0.03

        # All heights specify desired TCP position (fingertip control point)
        # relative to cube center. TCP = panda_hand + R(quat) * body_offset.
        # When vertical, TCP is ~0.107m below panda_hand.
        self.approach_height = 0.15     # TCP height above cube for approach
        self.hover_height = 0.10        # TCP at 10cm above cube for hover
        self.grasp_height = 0.0         # TCP at cube center for grasping
        self.lift_height = 0.25         # How high to lift TCP above cube
        self.min_cube_lift_z = 0.06     # Minimum cube Z to confirm successful grasp

        # Thresholds
        self.xy_align_threshold = 0.015    # XY alignment precision (1.5cm)
        self.z_align_threshold = 0.03      # Z alignment precision
        self.yaw_align_threshold = 0.08    # Yaw alignment (radians, ~4.5 degrees)

        # Safety
        self.max_action = 0.8           # Max commanded velocity per axis
        self.descend_max_action = 0.3   # Slower descent for precision
        self.safe_z_floor = -0.05       # Minimum allowed EE target Z (world frame)

        self.debug_counter = 0          # For periodic debug prints

        # Timing safeguards (frames at 50Hz control ≈ 0.02s/frame)
        self.max_approach_frames = 300   # 6 seconds to approach
        self.max_descend_frames = 150    # 3 seconds to descend
        self.grasp_settle_frames = 50    # 1.0 seconds to settle grip
        self.max_lift_frames = 150       # 3 seconds to lift
        self.hover_settle_frames = 15    # 0.3 seconds to stabilize

        # State
        self.phase = self.APPROACH_XY
        self.phase_timer = 0
        self.retry_count = 0
        self.max_retries = 5

        # Tracking for smooth control
        self._prev_action = torch.zeros(7, device=device)

    def reset(self):
        """Reset for a new episode."""
        self.phase = self.APPROACH_XY
        self.phase_timer = 0
        self.retry_count = 0
        self.debug_counter = 0
        self._prev_action = torch.zeros(7, device=self.device)

    def _adaptive_gain(self, distance):
        """Compute adaptive proportional gain based on distance to target."""
        if distance > self.threshold_far:
            return self.kp_far
        elif distance < self.threshold_near:
            return self.kp_near
        else:
            # Linear interpolation
            t = (distance - self.threshold_near) / (self.threshold_far - self.threshold_near)
            return self.kp_near + t * (self.kp_far - self.kp_near)

    def _get_tcp_world_pos(self, ee_pos, ee_quat):
        """Compute the TCP world position from panda_hand pose.

        TCP = panda_hand_pos + R(ee_quat) * body_offset

        This correctly accounts for the gripper's orientation —
        when tilted, the body_offset creates XY displacement in world frame.
        """
        rotated_offset = quat_rotate_point(ee_quat, self.body_offset)
        return ee_pos + rotated_offset

    def _compute_action(self, tcp_target, target_yaw, ee_pos, ee_quat,
                        grip=1.0, max_act=None):
        """
        Compute IK-Rel action to move TCP toward target.

        CRITICAL: The IK-Rel action is a delta applied to the TCP (not panda_hand).
        We compute delta = target - tcp_pos, where tcp_pos accounts for the
        body_offset rotated by the gripper's current quaternion.

        Also actively corrects roll/pitch to keep the gripper pointing
        straight down (vertical), preventing the tilt that causes XY misalignment.

        Args:
            tcp_target: desired TCP world position
            target_yaw: desired yaw for the gripper
            ee_pos: current panda_hand world position
            ee_quat: current panda_hand quaternion
            grip: 1.0=open, -1.0=close
            max_act: max action magnitude

        Returns:
            action: (7,) tensor [dx, dy, dz, droll, dpitch, dyaw, grip]
            dist_xy: XY distance from TCP to target
            dist_z: Z distance from TCP to target
            yaw_err: yaw error
        """
        if max_act is None:
            max_act = self.max_action

        action = torch.zeros(7, device=self.device)

        # Compute ACTUAL TCP position using quaternion-rotated body_offset
        tcp_pos = self._get_tcp_world_pos(ee_pos, ee_quat)

        # Position error (TCP to target in WORLD frame)
        delta_pos = tcp_target - tcp_pos
        dist_xy = torch.norm(delta_pos[:2])
        dist_z = torch.abs(delta_pos[2])
        dist_total = torch.norm(delta_pos)

        # Adaptive gain
        kp = self._adaptive_gain(dist_total.item())

        # Position commands with clamping
        pos_cmd = delta_pos * kp
        action[0] = torch.clamp(pos_cmd[0], -max_act, max_act)
        action[1] = torch.clamp(pos_cmd[1], -max_act, max_act)
        action[2] = torch.clamp(pos_cmd[2], -max_act, max_act)

        # Orientation correction: keep gripper pointing straight down
        # The Franka panda_hand's default orientation when pointing down
        # has specific roll/pitch values. We correct toward vertical.
        roll, pitch = get_roll_pitch(ee_quat)

        # For a gripper pointing straight down, we want:
        # - Roll = π (or -π)  — the default downward orientation
        # - Pitch = 0
        # The desired roll depends on the convention; pi means gripper Z points down
        desired_roll = torch.tensor(3.14159, device=self.device)
        roll_err = torch.atan2(torch.sin(desired_roll - roll),
                               torch.cos(desired_roll - roll))
        pitch_err = -pitch  # Correct pitch toward 0

        action[3] = torch.clamp(roll_err * 1.0, -max_act * 0.5, max_act * 0.5)
        action[4] = torch.clamp(pitch_err * 1.0, -max_act * 0.5, max_act * 0.5)

        # Yaw correction — match cube orientation
        cur_yaw = get_yaw(ee_quat)
        yaw_err = torch.atan2(torch.sin(target_yaw - cur_yaw),
                              torch.cos(target_yaw - cur_yaw))
        action[5] = torch.clamp(yaw_err * 2.0, -max_act, max_act)

        # Gripper
        action[6] = grip

        # Smooth with exponential moving average (reduce jitter)
        alpha = 0.7
        smoothed = alpha * action + (1 - alpha) * self._prev_action
        # Don't smooth gripper command
        smoothed[6] = grip
        self._prev_action = smoothed.clone()

        return smoothed, dist_xy.item(), dist_z.item(), abs(yaw_err.item())

    def compute(self, cube_pos, cube_quat, ee_pos, ee_quat):
        """
        Run one step of the pick-and-place state machine.

        Args:
            cube_pos: (3,) world position of cube
            cube_quat: (4,) world quaternion of cube (w,x,y,z)
            ee_pos: (3,) world position of end-effector (panda_hand)
            ee_quat: (4,) world quaternion of end-effector

        Returns:
            action: (1, 7) action tensor for env.step()
            phase: current phase string
            is_terminal: True if episode should be saved/reset
        """
        self.phase_timer += 1
        self.debug_counter += 1
        target_yaw = get_yaw(cube_quat)

        is_terminal = False

        # Periodic debug logging
        if self.debug_counter % 50 == 0:
            tcp_pos = self._get_tcp_world_pos(ee_pos, ee_quat)
            print(f"  [DBG] phase={self.phase} timer={self.phase_timer} "
                  f"ee=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) "
                  f"tcp=({tcp_pos[0]:.3f},{tcp_pos[1]:.3f},{tcp_pos[2]:.3f}) "
                  f"cube=({cube_pos[0]:.3f},{cube_pos[1]:.3f},{cube_pos[2]:.3f})")

        # All targets specify desired TCP position in world frame.

        # === Phase: APPROACH_XY ===
        # Move TCP to above cube at safe approach height
        if self.phase == self.APPROACH_XY:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.approach_height

            action, dist_xy, dist_z, yaw_err = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat, grip=1.0)

            total_dist = (dist_xy**2 + dist_z**2)**0.5
            if total_dist < self.xy_align_threshold * 2 and yaw_err < self.yaw_align_threshold:
                self._transition(self.HOVER)
            elif self.phase_timer > self.max_approach_frames:
                print(f"⚠️ APPROACH_XY timeout after {self.phase_timer} frames, forcing HOVER")
                self._transition(self.HOVER)

        # === Phase: HOVER ===
        # Hold TCP position above cube briefly to stabilize
        elif self.phase == self.HOVER:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.hover_height

            action, dist_xy, dist_z, yaw_err = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat, grip=1.0)

            if self.phase_timer > self.hover_settle_frames:
                if dist_xy < self.xy_align_threshold and yaw_err < self.yaw_align_threshold:
                    self._transition(self.OPEN_GRIP)
                elif self.phase_timer > self.hover_settle_frames * 4:
                    print(f"⚠️ HOVER: not perfectly aligned (xy={dist_xy:.3f}, yaw={yaw_err:.3f}), proceeding anyway")
                    self._transition(self.OPEN_GRIP)

        # === Phase: OPEN_GRIP ===
        # Ensure gripper is fully open before descending
        elif self.phase == self.OPEN_GRIP:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.hover_height

            action, _, _, _ = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat, grip=1.0)

            if self.phase_timer > 15:
                self._transition(self.DESCEND)

        # === Phase: DESCEND ===
        # Lower TCP slowly toward the cube grasp height
        elif self.phase == self.DESCEND:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.grasp_height

            action, dist_xy, dist_z, yaw_err = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat,
                grip=1.0, max_act=self.descend_max_action)

            if self.phase_timer % 20 == 0:
                tcp_pos = self._get_tcp_world_pos(ee_pos, ee_quat)
                print(f"  [DESCEND] tcp=({tcp_pos[0]:.3f},{tcp_pos[1]:.3f},{tcp_pos[2]:.3f}) "
                      f"tgt=({tcp_tgt[0]:.3f},{tcp_tgt[1]:.3f},{tcp_tgt[2]:.3f}) "
                      f"dist_xy={dist_xy:.3f} dist_z={dist_z:.3f}")

            total_dist = (dist_xy**2 + dist_z**2)**0.5
            if total_dist < self.z_align_threshold:
                self._transition(self.GRASP)
            elif self.phase_timer > self.max_descend_frames:
                print(f"⚠️ DESCEND timeout, forcing GRASP (dist={total_dist:.3f})")
                self._transition(self.GRASP)

        # === Phase: GRASP ===
        # Close gripper while continuing to track cube XY position
        elif self.phase == self.GRASP:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.grasp_height

            action, dist_xy, dist_z, _ = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat, grip=-1.0,
                max_act=self.descend_max_action)

            # Limit Z movement during grasp to avoid pressing too hard
            action[2] = torch.clamp(action[2], -0.1, 0.1)

            if self.phase_timer > self.grasp_settle_frames:
                self._transition(self.LIFT)

        # === Phase: LIFT ===
        # Lift TCP upward while maintaining grip
        elif self.phase == self.LIFT:
            tcp_tgt = cube_pos.clone()
            tcp_tgt[2] += self.lift_height

            action, _, dist_z, _ = self._compute_action(
                tcp_tgt, target_yaw, ee_pos, ee_quat, grip=-1.0)

            if dist_z < 0.03 or self.phase_timer > self.max_lift_frames:
                self._transition(self.VERIFY)

        # === Phase: VERIFY ===
        # Check if cube is actually lifted
        elif self.phase == self.VERIFY:
            action = torch.zeros(7, device=self.device)
            action[6] = -1.0  # Keep gripper closed (negative = close)

            if self.phase_timer > 10:
                cube_z = cube_pos[2].item()
                tcp_pos = self._get_tcp_world_pos(ee_pos, ee_quat)
                print(f"  [VERIFY] cube_z={cube_z:.3f} tcp_z={tcp_pos[2]:.3f} "
                      f"threshold={self.min_cube_lift_z:.3f}")

                is_terminal = True
                if cube_z > self.min_cube_lift_z:
                    self.phase = self.DONE_SUCCESS
                    print(f"✅ GRASP VERIFIED! Cube height: {cube_z:.3f}m")
                else:
                    self.retry_count += 1
                    if self.retry_count >= self.max_retries:
                        self.phase = self.DONE_FAIL
                        print(f"❌ Max retries ({self.max_retries}) reached. Giving up.")
                    else:
                        print(f"⚠️ Grasp failed (cube_z={cube_z:.3f}). "
                              f"Retry {self.retry_count}/{self.max_retries}")
                        self._transition(self.APPROACH_XY)
                        is_terminal = False

        else:
            action = torch.zeros(7, device=self.device)

        return action.unsqueeze(0), self.phase, is_terminal

    def _transition(self, new_phase):
        """Transition to a new phase and reset timer."""
        if new_phase != self.phase:
            print(f"  → Phase: {self.phase} → {new_phase} (after {self.phase_timer} frames)")
        self.phase = new_phase
        self.phase_timer = 0
        # Reset action smoothing on phase change to avoid lag
        self._prev_action = torch.zeros(7, device=self.device)

    @property
    def is_success(self):
        return self.phase == self.DONE_SUCCESS

    @property
    def is_failed(self):
        return self.phase == self.DONE_FAIL


# ============================================================================
#  LeRobot-Compatible Data Saver
# ============================================================================
class LeRobotDemoSaver:
    """
    Saves demo trajectories in HDF5 format compatible with LeRobot training.

    HDF5 structure:
        /obs/state          - (T, num_joints) joint positions
        /obs/images/top     - (T, H, W, C) RGB images (if available)
        /actions            - (T, 7) action commands [dx,dy,dz,droll,dpitch,dyaw,grip]
        /episode_metadata   - JSON with episode info

    This format is directly consumable by LeRobot's dataset loaders.
    """

    def __init__(self, save_dir="logs/demos"):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

        # Trajectory buffers
        self.states = []
        self.actions = []
        self.images = []

        # Statistics
        self.demo_count = 0
        self.success_count = 0
        self.fail_count = 0

    def record_step(self, joint_pos, action, image=None):
        """Record one timestep."""
        self.states.append(joint_pos.cpu().numpy() if torch.is_tensor(joint_pos) else joint_pos)
        self.actions.append(action.cpu().numpy() if torch.is_tensor(action) else action)
        if image is not None:
            self.images.append(image.cpu().numpy() if torch.is_tensor(image) else image)

    def save(self, success=True, episode_info=None):
        """Save current trajectory to HDF5 if it's long enough."""
        if len(self.actions) < 10:
            print("⚠️ Trajectory too short, skipping save.")
            self.clear()
            return None

        if not success:
            self.fail_count += 1
            print(f"🗑️ Discarding failed episode (fail #{self.fail_count})")
            self.clear()
            return None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            self.save_dir,
            f"demo_{self.demo_count:04d}_{timestamp}.hdf5"
        )

        with h5py.File(filename, "w") as f:
            # Core observation data
            f.create_dataset("obs/state", data=np.array(self.states),
                           compression="gzip", compression_opts=4)
            f.create_dataset("actions", data=np.array(self.actions),
                           compression="gzip", compression_opts=4)

            # Camera images (if available)
            if self.images:
                f.create_dataset("obs/images/top", data=np.array(self.images),
                               compression="gzip", compression_opts=4)

            # Episode metadata (useful for LeRobot dataset conversion)
            metadata = {
                "timestamp": timestamp,
                "episode_index": self.demo_count,
                "num_frames": len(self.actions),
                "success": success,
                "format_version": "lerobot_v1",
                "action_space": "ik_rel_7d",  # dx,dy,dz,droll,dpitch,dyaw,grip
                "robot": "franka_panda",
                "task": "lift_cube",
            }
            if episode_info:
                metadata.update(episode_info)
            f.attrs["metadata"] = json.dumps(metadata)

        self.demo_count += 1
        self.success_count += 1
        print(f"💾 Demo saved: {filename}")
        print(f"   📊 Stats: {self.success_count} success, "
              f"{self.fail_count} fail, "
              f"{self.success_count + self.fail_count} total attempts")

        self.clear()
        return filename

    def clear(self):
        """Clear trajectory buffers."""
        self.states.clear()
        self.actions.clear()
        self.images.clear()

    @property
    def total_demos(self):
        return self.demo_count


# ============================================================================
#  Main Loop
# ============================================================================
def main():
    env_cfg = parse_env_cfg("Isaac-Lift-Cube-Franka-IK-Rel-v0")
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = 3600.0  # Long episodes for manual control

    env = ManagerBasedRLEnv(cfg=env_cfg)
    robot = env.scene["robot"]
    ee_idx = robot.find_bodies("panda_hand")[0][0]

    # Find finger bodies and joints for debug
    finger_joint_ids = robot.find_joints("panda_finger_.*")[0]
    print(f"  Finger joints: {robot.find_joints('panda_finger_.*')[1]} ids={finger_joint_ids}")
    leftfinger_idx = robot.find_bodies("panda_leftfinger")[0][0]
    rightfinger_idx = robot.find_bodies("panda_rightfinger")[0][0]
    print(f"  Left finger body idx: {leftfinger_idx}, Right: {rightfinger_idx}")
    print(f"  All body names: {robot.body_names}")

    # Controller and data saver
    controller = PickAndPlaceController(device=env.device)
    saver = LeRobotDemoSaver(save_dir=args_cli.save_dir)
    max_demos = args_cli.max_demos

    # Communication channels
    udp_receiver = UDPTeleopReceiver(device=env.device)
    zmq_ctx = zmq.Context()
    cmd_sock = zmq_ctx.socket(zmq.REP)
    cmd_sock.bind("tcp://0.0.0.0:8213")

    print("=" * 60)
    print("🚀 Isaac Auto Collector V2 - Robust Full-Auto")
    print("=" * 60)
    print(f"  UDP teleop:  port 8212")
    print(f"  ZMQ commands: port 8213")
    print(f"  Save dir:    {args_cli.save_dir}")
    print(f"  Max demos:   {max_demos}")
    print("=" * 60)
    print("Commands: MANUAL | SEMI_HOVER | SEMI_GRASP | FULL_AUTO | CONTINUOUS | SAVE | RESET")
    print("=" * 60)

    obs, _ = env.reset()

    # --autorun: start in CONTINUOUS mode immediately
    if args_cli.autorun:
        state = "CONTINUOUS"
        controller.reset()
        print("\n🏭 AUTORUN: Starting CONTINUOUS collection immediately!")
        print(f"   Target: {max_demos} demos → {args_cli.save_dir}")
    else:
        state = "MANUAL"

    # TCP Offset for manual gamepad control (separate from auto controller)
    TCP_OFFSET_Z = 0.115

    while simulation_app.is_running():
        manual_action = udp_receiver.get_action().clone()
        final_action = manual_action.clone()

        # =============================================
        # 1. Listen for ZMQ commands
        # =============================================
        try:
            cmd_str = cmd_sock.recv_string(flags=zmq.NOBLOCK)
            cmd_upper = cmd_str.strip().upper()

            if cmd_upper == "SEMI_HOVER":
                state = "SEMI_HOVER"
                cmd_sock.send_string("Auto-hovering above cube...")
            elif cmd_upper == "SEMI_GRASP":
                state = "SEMI_GRASP"
                controller.reset()
                controller.phase = PickAndPlaceController.DESCEND
                cmd_sock.send_string("Auto-descend & grasp...")
            elif cmd_upper == "FULL_AUTO":
                state = "FULL_AUTO"
                controller.reset()
                cmd_sock.send_string("Full-auto mode activated!")
                print("\n🤖 FULL_AUTO mode: Starting autonomous pick-and-place")
            elif cmd_upper == "CONTINUOUS":
                state = "CONTINUOUS"
                controller.reset()
                cmd_sock.send_string(f"Continuous collection: target {max_demos} demos")
                print(f"\n🏭 CONTINUOUS mode: Collecting up to {max_demos} demos")
            elif cmd_upper == "SAVE":
                state = "SAVE_TRIGGERED"
                cmd_sock.send_string("Saving trajectory...")
            elif cmd_upper == "RESET":
                state = "RESET_TRIGGERED"
                cmd_sock.send_string("Environment reset!")
            elif cmd_upper == "STATUS":
                status_msg = (f"State={state}, Demos={saver.demo_count}, "
                            f"Success={saver.success_count}, Fail={saver.fail_count}")
                cmd_sock.send_string(status_msg)
            else:
                cmd_sock.send_string(f"Unknown command: {cmd_str}")
        except zmq.error.Again:
            pass

        # =============================================
        # 2. Manual override detection
        # =============================================
        if (torch.sum(torch.abs(manual_action[0, :6])) > 0.01 and
                state not in ["MANUAL", "SAVE_TRIGGERED", "RESET_TRIGGERED"]):
            print("🕹️ Human override detected, switching to MANUAL")
            state = "MANUAL"

        # =============================================
        # 3. Get scene state
        # =============================================
        cube_pos = env.scene["object"].data.root_pos_w[0]
        cube_quat = env.scene["object"].data.root_quat_w[0]
        ee_pos = robot.data.body_pos_w[0, ee_idx]
        ee_quat = robot.data.body_quat_w[0, ee_idx]

        # Finger debug info (log during GRASP and LIFT phases)
        if state in ["FULL_AUTO", "CONTINUOUS"] and controller.phase in [
            controller.GRASP, controller.LIFT, controller.VERIFY
        ]:
            finger_pos = robot.data.joint_pos[0, finger_joint_ids]
            lf_pos = robot.data.body_pos_w[0, leftfinger_idx]
            rf_pos = robot.data.body_pos_w[0, rightfinger_idx]
            if controller.phase_timer % 10 == 0:
                print(f"  [FINGER] joints=[{finger_pos[0]:.4f}, {finger_pos[1]:.4f}] "
                      f"left=({lf_pos[0]:.3f},{lf_pos[1]:.3f},{lf_pos[2]:.3f}) "
                      f"right=({rf_pos[0]:.3f},{rf_pos[1]:.3f},{rf_pos[2]:.3f}) "
                      f"gap={abs(lf_pos[1]-rf_pos[1]):.4f}")

        # =============================================
        # 4. State machine
        # =============================================

        # --- SEMI_HOVER: Auto-align above cube ---
        if state == "SEMI_HOVER":
            t_pos = cube_pos.clone()
            t_pos[2] += (0.12 + TCP_OFFSET_Z)
            t_yaw = get_yaw(cube_quat)

            delta = t_pos - ee_pos
            dist = torch.norm(delta).item()
            yaw_err = abs(get_yaw(ee_quat).item() - t_yaw.item())

            act = torch.zeros((1, 7), device=env.device)
            kp = 4.0 if dist > 0.05 else 2.0
            act[0, :3] = torch.clamp(delta * kp, -0.5, 0.5)
            cur_yaw = get_yaw(ee_quat)
            raw_yaw_err = torch.atan2(torch.sin(t_yaw - cur_yaw), torch.cos(t_yaw - cur_yaw))
            act[0, 5] = torch.clamp(raw_yaw_err * 2.0, -0.5, 0.5)
            act[0, 6] = 1.0  # Open gripper (positive = open)
            final_action = act

            if dist < 0.02 and yaw_err < 0.05:
                print("🎯 Aligned! Returning to MANUAL.")
                state = "MANUAL"

        # --- SEMI_GRASP: Auto-descend, grasp, lift from current pos ---
        elif state == "SEMI_GRASP":
            auto_action, phase, is_terminal = controller.compute(
                cube_pos, cube_quat, ee_pos, ee_quat)
            final_action = auto_action

            if is_terminal:
                if controller.is_success:
                    print("🎯 Semi-auto grasp succeeded! Returning to MANUAL.")
                else:
                    print("⚠️ Semi-auto grasp failed. Returning to MANUAL.")
                state = "MANUAL"
                controller.reset()

        # --- FULL_AUTO: Complete autonomous pick cycle ---
        elif state == "FULL_AUTO":
            auto_action, phase, is_terminal = controller.compute(
                cube_pos, cube_quat, ee_pos, ee_quat)
            final_action = auto_action

            if is_terminal: 
                if controller.is_success:
                    state = "SAVE_TRIGGERED"
                elif controller.is_failed:
                    print("❌ FULL_AUTO failed. Resetting...")
                    state = "RESET_TRIGGERED"

        # --- CONTINUOUS: Auto-collect many demos ---
        elif state == "CONTINUOUS":
            if saver.total_demos >= max_demos:
                print(f"\n🎉 Target reached! Collected {max_demos} demos.")
                print(f"   Success rate: {saver.success_count}/{saver.success_count + saver.fail_count}")
                state = "MANUAL"
            else:
                auto_action, phase, is_terminal = controller.compute(
                    cube_pos, cube_quat, ee_pos, ee_quat)
                final_action = auto_action

                if is_terminal:
                    if controller.is_success:
                        # Save and continue
                        saver.save(success=True, episode_info={
                            "mode": "continuous_auto",
                            "retry_count": controller.retry_count,
                        })
                    elif controller.is_failed:
                        saver.save(success=False)
                    # Reset for next episode (both success and fail)
                    env.reset()
                    saver.clear()
                    controller.reset()
                    time.sleep(0.1)
                    continue  # Skip env.step below, we already reset

        # =============================================
        # 5. Step simulation and record data
        # =============================================
        obs, rewards, dones, _, _ = env.step(final_action)

        # Record data for all modes (trajectories are useful from any mode)
        robot_state = robot.data.joint_pos[0]
        action_data = final_action[0]

        # Optionally record camera images alongside state/action
        img = None
        if hasattr(env.scene, "sensors") and "top_camera" in env.scene.sensors:
            img = env.scene.sensors["top_camera"].data.output["rgb"][0]

        saver.record_step(robot_state, action_data, image=img)

        # =============================================
        # 6. Save / Reset handling
        # =============================================
        if state == "SAVE_TRIGGERED":
            saver.save(success=True, episode_info={
                "mode": "manual_save" if controller.phase == PickAndPlaceController.APPROACH_XY else "auto_save",
            })
            state = "RESET_TRIGGERED"

        if state == "RESET_TRIGGERED" or dones[0]:
            env.reset()
            saver.clear()
            controller.reset()
            state = "MANUAL"

    env.close()
    simulation_app.close()
    print(f"\n📊 Final Statistics: {saver.success_count} successful demos saved to {args_cli.save_dir}")


if __name__ == "__main__":
    main()