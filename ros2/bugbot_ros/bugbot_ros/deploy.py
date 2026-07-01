"""Shared sim-to-real core for BugBot: observation assembly, normalisation, and
policy inference. Reused by the ROS nodes so the deployment logic is tested once
and kept identical to the simulation.

The constants and observation layout below MUST match the simulation env
(``simulation/bugbot_tasks/bugbot_env_cfg.py`` / ``bugbot_env.py``): a policy
trained there consumes exactly this observation vector.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .mixing import normalise_wheels, twist_to_wheels

# --- normalisation (keep in step with BugBotEnvCfg) ---
MAX_LIN_SPEED = 0.5   # m/s at |twist| = 1
MAX_YAW_RATE = 3.0    # rad/s at |rot| = 1
N_WHEELS = 4
OBS_DIM = 10          # cmd(3) + meas_lin(2) + meas_yaw(1) + prev_action(4)


@dataclass
class RobotSensors:
    """Latest sensor readings in SI units, body frame."""
    lin_vel_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))  # m/s (vx, vy)
    yaw_rate: float = 0.0                                                # rad/s


class ObservationBuilder:
    """Assemble the 10-dim observation the policy was trained on.

    Layout (all normalised): [cmd_vx, cmd_vy, cmd_wz, meas_vx, meas_vy, meas_wz,
    prev_action(4)].
    """

    def __init__(self):
        self.prev_action = np.zeros(N_WHEELS, dtype=np.float32)

    def build(self, cmd_twist, sensors: RobotSensors) -> np.ndarray:
        cmd = np.asarray(cmd_twist, dtype=np.float32)  # (vx, vy, wz) in SI
        obs = np.empty(OBS_DIM, dtype=np.float32)
        obs[0] = cmd[0] / MAX_LIN_SPEED
        obs[1] = cmd[1] / MAX_LIN_SPEED
        obs[2] = cmd[2] / MAX_YAW_RATE
        obs[3] = sensors.lin_vel_xy[0] / MAX_LIN_SPEED
        obs[4] = sensors.lin_vel_xy[1] / MAX_LIN_SPEED
        obs[5] = sensors.yaw_rate / MAX_YAW_RATE
        obs[6:10] = self.prev_action
        return obs

    def set_prev_action(self, action) -> None:
        self.prev_action = np.asarray(action, dtype=np.float32).reshape(N_WHEELS)


class PolicyController:
    """Loads a TorchScript-exported policy and maps observations to wheel commands.

    Export the trained rsl_rl policy to TorchScript first (rsl_rl's exporter, or
    the sim's play path). If ``policy_path`` is None, acts as a zero policy
    (robot holds still), which is a safe default for bring-up.
    """

    def __init__(self, policy_path: Optional[str] = None, device: str = "cpu"):
        self.builder = ObservationBuilder()
        self._policy = None
        self._device = device
        if policy_path:
            import torch  # imported lazily so the twist bridge needs no torch
            self._torch = torch
            self._policy = torch.jit.load(policy_path, map_location=device).eval()

    def act(self, cmd_twist, sensors: RobotSensors) -> np.ndarray:
        """Return four wheel commands in [-1, 1] (BR, FR, BL, FL)."""
        obs = self.builder.build(cmd_twist, sensors)
        if self._policy is None:
            action = np.zeros(N_WHEELS, dtype=np.float32)
        else:
            with self._torch.inference_mode():
                t = self._torch.from_numpy(obs).float().unsqueeze(0).to(self._device)
                action = self._policy(t).squeeze(0).cpu().numpy()
        action = np.clip(action, -1.0, 1.0)
        self.builder.set_prev_action(action)
        return action


def twist_to_wheel_cmds(vx: float, vy_right: float, wz: float) -> np.ndarray:
    """Direct (non-learned) mapping: body twist -> normalised wheel commands,
    capped like the firmware. ``vy_right`` is lateral with right positive."""
    wheels = twist_to_wheels(vx / MAX_LIN_SPEED, vy_right / MAX_LIN_SPEED, wz / MAX_YAW_RATE)
    return normalise_wheels(wheels, 1.0)
