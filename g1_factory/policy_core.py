"""Simulator-independent core of Unitree's pretrained G1 12-DOF walking policy.

Builds the 47-dim observation and runs the TorchScript policy at 50 Hz. Used by
the MuJoCo wrapper (g1_factory.locomotion) and the Isaac Sim node
(g1_factory.isaac), so both simulators feed the policy exactly the same inputs.
Observation layout and scales follow unitree_rl_gym deploy_mujoco.py at commit
276801e46c5d433564f24658bac64f254b7d2d4b (BSD-3-Clause, see locomotion.py).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
)


def projected_gravity(quaternion) -> np.ndarray:
    """World gravity (0, 0, -1) expressed in the floating base frame, wxyz."""
    qw, qx, qy, qz = quaternion
    return np.array(
        [2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx),
         1 - 2 * (qw * qw + qz * qz)],
        dtype=np.float32,
    )


class G1PolicyCore:
    """Observation + policy. Inputs are plain arrays in the policy's joint order.

    act(base_quat_wxyz, base_ang_vel_body, q, dq, command, sim_time) -> PD targets.
    PD gains (kp, kd) are exposed for the simulator's joint drives.
    """

    joint_names = JOINT_NAMES
    policy_hz = 50.0

    def __init__(self, upstream_root: str | Path):
        import torch

        self._torch = torch
        root = Path(upstream_root).resolve()
        config_path = root / "deploy/deploy_mujoco/configs/g1.yaml"
        self.policy_path = root / "deploy/pre_train/g1/motion.pt"
        if not config_path.is_file() or not self.policy_path.is_file():
            raise FileNotFoundError("G1 assets missing; run scripts/fetch_assets.py first.")
        self.config = yaml.safe_load(config_path.read_text())
        if (self.config["num_obs"], self.config["num_actions"]) != (47, 12):
            raise ValueError("Expected the official 47-observation, 12-action G1 policy.")
        self.default_angles = self._vector("default_angles", 12)
        self.kp = self._vector("kps", 12)
        self.kd = self._vector("kds", 12)
        self.cmd_scale = self._vector("cmd_scale", 3)
        self.action_scale = float(self.config["action_scale"])
        # Tiny recurrent policy: one CPU thread avoids inference thread overhead.
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(self.policy_path), map_location="cpu").eval()
        self.action = np.zeros(12, dtype=np.float32)
        self.observation = np.zeros(47, dtype=np.float32)
        self.target_positions = self.default_angles.copy()

    def _vector(self, name: str, size: int) -> np.ndarray:
        value = np.asarray(self.config[name], dtype=np.float32)
        if value.shape != (size,) or not np.all(np.isfinite(value)):
            raise ValueError(f"Invalid official G1 config field: {name}.")
        return value

    def reset(self) -> None:
        self.action.fill(0)
        self.observation.fill(0)
        self.target_positions[:] = self.default_angles
        if hasattr(self.policy, "reset_memory"):
            self.policy.reset_memory()
        else:
            # Reload also handles alternative recurrent exports without a reset API.
            self.policy = self._torch.jit.load(str(self.policy_path), map_location="cpu").eval()

    def act(self, base_quat_wxyz, base_ang_vel_body, q, dq, command, sim_time: float) -> np.ndarray:
        obs = self.observation
        obs[0:3] = np.asarray(base_ang_vel_body, dtype=np.float32) * float(self.config["ang_vel_scale"])
        obs[3:6] = projected_gravity(base_quat_wxyz)
        obs[6:9] = np.asarray(command, dtype=np.float32) * self.cmd_scale
        obs[9:21] = (np.asarray(q, dtype=np.float32) - self.default_angles) * float(self.config["dof_pos_scale"])
        obs[21:33] = np.asarray(dq, dtype=np.float32) * float(self.config["dof_vel_scale"])
        obs[33:45] = self.action
        phase = (sim_time % 0.8) / 0.8
        obs[45:47] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
        if not np.all(np.isfinite(obs)):
            raise RuntimeError("Nonfinite G1 state; terminate this episode.")
        # inference_mode would freeze mutable hidden-state tensors of some exports.
        with self._torch.no_grad():
            prediction = self.policy(self._torch.from_numpy(obs).unsqueeze(0))
        action = prediction.detach().cpu().numpy().reshape(-1)
        if action.shape != (12,) or not np.all(np.isfinite(action)):
            raise RuntimeError("G1 policy returned invalid actions.")
        self.action[:] = action
        self.target_positions[:] = self.default_angles + self.action_scale * self.action
        return self.target_positions
