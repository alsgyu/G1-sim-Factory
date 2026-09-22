"""CPU inference for Unitree's matched G1 12-DOF walking policy.

Observation construction and PD control adapt unitree_rl_gym's deploy_mujoco.py
at commit 276801e46c5d433564f24658bac64f254b7d2d4b. The robot moves exclusively
through joint torques and MuJoCo contact dynamics after an episode reset.

BSD 3-Clause License
Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import yaml


JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
)


def projected_gravity(quaternion: np.ndarray) -> np.ndarray:
    """World gravity (0, 0, -1) expressed in the floating base frame, wxyz."""
    qw, qx, qy, qz = quaternion
    return np.array(
        [2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx),
         1 - 2 * (qw * qw + qz * qz)],
        dtype=np.float32,
    )


class G1Locomotion:
    """Run one 2 ms physics step per ``step([vx, vy, yaw_rate])`` call.

    Linear commands are in the robot frame (m/s); yaw rate is rad/s. The
    TorchScript policy runs at 50 Hz, with PD torques applied at 500 Hz. Use
    reset() between episodes to clear the LSTM state. The bundled policy has
    fixed arms/waist and is not compatible with a freely articulated 29-DOF G1.
    """

    joint_names = JOINT_NAMES

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 upstream_root: str | Path):
        import torch

        self._torch = torch
        self.model, self.data = model, data
        self.upstream_root = Path(upstream_root).resolve()
        config_path = self.upstream_root / "deploy/deploy_mujoco/configs/g1.yaml"
        self.policy_path = self.upstream_root / "deploy/pre_train/g1/motion.pt"
        if not config_path.is_file() or not self.policy_path.is_file():
            raise FileNotFoundError("G1 assets missing; run scripts/fetch_assets.py first.")
        self.config = yaml.safe_load(config_path.read_text())
        self.dt = float(self.config["simulation_dt"])
        self.decimation = int(self.config["control_decimation"])
        self.control_dt = self.dt * self.decimation
        if (self.config["num_obs"], self.config["num_actions"]) != (47, 12):
            raise ValueError("Expected the official 47-observation, 12-action G1 policy.")
        if not np.isclose(self.dt, 0.002) or self.decimation != 10:
            raise ValueError("The supported G1 policy requires dt=0.002, decimation=10.")
        if not np.isclose(model.opt.timestep, self.dt):
            raise ValueError(f"Scene timestep must be {self.dt}, got {model.opt.timestep}.")
        self.default_angles = self._config_vector("default_angles", 12)
        self.kp = self._config_vector("kps", 12)
        self.kd = self._config_vector("kds", 12)
        self.cmd_scale = self._config_vector("cmd_scale", 3)
        self.action_scale = float(self.config["action_scale"])
        self._map_robot()
        # Tiny recurrent policy: one CPU thread avoids inference thread overhead.
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(self.policy_path), map_location="cpu").eval()
        self.action = np.zeros(12, dtype=np.float32)
        self.target_positions = self.default_angles.copy()
        self.observation = np.zeros(47, dtype=np.float32)
        self.counter = 0

    def _config_vector(self, name: str, size: int) -> np.ndarray:
        value = np.asarray(self.config[name], dtype=np.float32)
        if value.shape != (size,) or not np.all(np.isfinite(value)):
            raise ValueError(f"Invalid official G1 config field: {name}.")
        return value

    def _map_robot(self) -> None:
        """Map by names so movable factory objects cannot shift robot indexing."""
        model = self.model
        if model.nu != 12:
            raise ValueError(f"Expected exactly 12 G1 motors; scene has {model.nu}.")
        joint_ids, actuator_ids = [], []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if jid < 0 or aid < 0:
                raise ValueError(f"Matched g1_12dof.xml joint/motor missing: {name}.")
            if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"G1 joint must be a hinge: {name}.")
            if (model.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_trnid[aid, 0] != jid):
                raise ValueError(f"Wrong G1 motor transmission for {name}.")
            if not np.allclose(model.actuator_gear[aid], [1, 0, 0, 0, 0, 0]):
                raise ValueError(f"Expected unit-gear torque motor for {name}.")
            joint_ids.append(jid)
            actuator_ids.append(aid)
        self.joint_ids = np.asarray(joint_ids, dtype=int)
        self.actuator_ids = np.asarray(actuator_ids, dtype=int)
        self.qpos_indices = model.jnt_qposadr[self.joint_ids].copy()
        self.dof_indices = model.jnt_dofadr[self.joint_ids].copy()
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        if root < 0 or model.jnt_type[root] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("The matched G1 floating_base_joint is required.")
        self.base_qpos = int(model.jnt_qposadr[root])
        self.base_dof = int(model.jnt_dofadr[root])

    def reset(self, xy=(0.0, 0.0), yaw: float = 0.0) -> None:
        """Initialize an episode at a stand pose; subsequent motion is dynamic."""
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.all(np.isfinite(xy)) or not np.isfinite(yaw):
            raise ValueError("reset requires finite xy[2] and yaw.")
        mujoco.mj_resetData(self.model, self.data)
        b = self.base_qpos
        self.data.qpos[b:b + 2] = xy
        self.data.qpos[b + 3:b + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.data.qpos[self.qpos_indices] = self.default_angles
        self.counter = 0
        self.action.fill(0)
        self.target_positions[:] = self.default_angles
        self.observation.fill(0)
        if hasattr(self.policy, "reset_memory"):
            self.policy.reset_memory()
        else:
            # Reload also handles alternative recurrent exports without a reset API.
            self.policy = self._torch.jit.load(str(self.policy_path), map_location="cpu").eval()
        mujoco.mj_forward(self.model, self.data)

    def _update_policy(self, command: np.ndarray) -> None:
        d, b, v = self.data, self.base_qpos, self.base_dof
        obs = self.observation
        obs[0:3] = d.qvel[v + 3:v + 6] * float(self.config["ang_vel_scale"])
        obs[3:6] = projected_gravity(d.qpos[b + 3:b + 7])
        obs[6:9] = command * self.cmd_scale
        obs[9:21] = ((d.qpos[self.qpos_indices] - self.default_angles)
                      * float(self.config["dof_pos_scale"]))
        obs[21:33] = d.qvel[self.dof_indices] * float(self.config["dof_vel_scale"])
        obs[33:45] = self.action
        phase = (self.counter * self.dt % 0.8) / 0.8
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

    def step(self, command) -> None:
        """Apply PD torques, step physics, and update policy every tenth step."""
        command = np.asarray(command, dtype=np.float32)
        if command.shape != (3,) or not np.all(np.isfinite(command)):
            raise ValueError("command must contain finite [vx, vy, yaw_rate].")
        torque = ((self.target_positions - self.data.qpos[self.qpos_indices]) * self.kp
                  - self.data.qvel[self.dof_indices] * self.kd)
        if not np.all(np.isfinite(torque)):
            raise RuntimeError("Nonfinite G1 torque; terminate this episode.")
        # XML joint actuatorfrcrange enforces the original motor force limits.
        self.data.ctrl[self.actuator_ids] = torque
        mujoco.mj_step(self.model, self.data)
        self.counter += 1
        if self.counter % self.decimation == 0:
            self._update_policy(command)
