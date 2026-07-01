"""BugBot holonomic velocity-tracking DirectRLEnv (Isaac Lab v2.3.2).

Drive model
-----------
Real omni-wheel roller contact is not simulated. Instead the four wheel commands
are turned into an *actual* body twist through the firmware X-omni mix plus a
per-env chaos model, and that twist is written to the robot root. This puts the
interesting physics (the chaotic coupling the sensors must correct for) exactly
where BugBot's real difficulty lives, and stays fast for thousands of envs.

STATUS: validated on Isaac Lab v2.3.2 (RTX 4070) -- the env loads, trains, and
the policy learns to track the commanded twist (mean reward ~43 -> ~555 over 150
iterations). The remaining `VALIDATE` notes are realism-tuning knobs (the
drive-injection approach, chaos/reward scales), not correctness blockers.
"""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from .bugbot_env_cfg import BugBotEnvCfg
from .mixing import MIX, UNMIX


class BugBotEnv(DirectRLEnv):
    cfg: BugBotEnvCfg

    def __init__(self, cfg: BugBotEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        dev = self.device
        # (4,3) and (3,4) mix matrices as tensors; UNMIX recovers twist from wheels.
        self._mix = torch.tensor(MIX, dtype=torch.float32, device=dev)
        self._unmix = torch.tensor(UNMIX, dtype=torch.float32, device=dev)

        n = self.num_envs
        self.actions = torch.zeros(n, 4, device=dev)
        self.prev_actions = torch.zeros(n, 4, device=dev)
        self.commands = torch.zeros(n, 3, device=dev)       # normalised (vx, vy, wz) in [-1,1]
        self._actual_twist = torch.zeros(n, 3, device=dev)  # latency-filtered normalised twist
        self._wheel_gain = torch.ones(n, 4, device=dev)     # per-env, per-wheel chaos gain

        self._resample_chaos(torch.arange(n, device=dev))

    # ------------------------------------------------------------------ scene
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        self.scene.clone_environments(copy_from_source=False)
        self.scene.articulations["robot"] = self.robot
        light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light.func("/World/Light", light)

    # ------------------------------------------------------------- step logic
    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clamp(-1.0, 1.0)

    def _apply_action(self):
        c = self.cfg
        # 1) per-wheel slip/gain, then recover the (normalised) body twist
        wheels = self.actions * self._wheel_gain
        twist = wheels @ self._unmix.T                      # (n,3): lon, lat, rot

        # 2) translation<->rotation coupling (the drive is not clean)
        k = c.chaos_coupling
        coupled = torch.empty_like(twist)
        coupled[:, 0] = twist[:, 0] + k * twist[:, 2]
        coupled[:, 1] = twist[:, 1] - k * twist[:, 2]
        coupled[:, 2] = twist[:, 2] + k * (twist[:, 0] - twist[:, 1])

        # 3) vibration (white noise) + 4) first-order actuation latency
        coupled = coupled + torch.randn_like(coupled) * c.chaos_vibration_std
        a = c.chaos_latency
        self._actual_twist = a * self._actual_twist + (1.0 - a) * coupled

        # scale normalised twist to physical units
        vx = self._actual_twist[:, 0] * c.max_lin_speed
        vy = self._actual_twist[:, 1] * c.max_lin_speed
        wz = self._actual_twist[:, 2] * c.max_yaw_rate

        # rotate body-frame (vx,vy) into world frame using current yaw
        yaw = self._yaw()
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        root_vel = torch.zeros(self.num_envs, 6, device=self.device)
        root_vel[:, 0] = cy * vx - sy * vy
        root_vel[:, 1] = sy * vx + cy * vy
        root_vel[:, 5] = wz
        # VALIDATE: writing root velocity each step overrides contact dynamics for
        # the base; intended for this planar drive model.
        self.robot.write_root_velocity_to_sim(root_vel)

        # spin the visible wheels for feedback (not load-bearing)
        self.robot.set_joint_velocity_target(wheels * c.wheel_spin_scale)

    # ----------------------------------------------------------- observations
    def _get_observations(self) -> dict:
        c = self.cfg
        lin_b = self.robot.data.root_lin_vel_b[:, :2]       # measured body vx, vy
        yaw_rate = self.robot.data.root_ang_vel_b[:, 2:3]   # measured yaw rate
        meas_lin = lin_b + torch.randn_like(lin_b) * c.obs_linvel_noise_std
        meas_yaw = yaw_rate + torch.randn_like(yaw_rate) * c.obs_yawrate_noise_std

        obs = torch.cat(
            [
                self.commands,                              # (n,3) normalised command
                meas_lin / c.max_lin_speed,                 # (n,2) normalised measured lin vel
                meas_yaw / c.max_yaw_rate,                  # (n,1) normalised measured yaw rate
                self.prev_actions,                          # (n,4)
            ],
            dim=-1,
        )
        self.prev_actions = self.actions.clone()
        return {"policy": obs}

    # ---------------------------------------------------------------- rewards
    def _get_rewards(self) -> torch.Tensor:
        c = self.cfg
        cmd_vx = self.commands[:, 0] * c.max_lin_speed
        cmd_vy = self.commands[:, 1] * c.max_lin_speed
        cmd_wz = self.commands[:, 2] * c.max_yaw_rate

        lin_b = self.robot.data.root_lin_vel_b[:, :2]
        yaw_rate = self.robot.data.root_ang_vel_b[:, 2]

        lin_err = (cmd_vx - lin_b[:, 0]) ** 2 + (cmd_vy - lin_b[:, 1]) ** 2
        yaw_err = (cmd_wz - yaw_rate) ** 2

        r_lin = torch.exp(-lin_err / c.rew_tracking_sigma) * c.rew_track_lin
        r_yaw = torch.exp(-yaw_err / c.rew_tracking_sigma) * c.rew_track_yaw
        r_effort = self.actions.abs().sum(dim=-1) * c.rew_effort
        r_rate = ((self.actions - self.prev_actions) ** 2).sum(dim=-1) * c.rew_action_rate
        return r_lin + r_yaw + r_effort + r_rate

    # ------------------------------------------------------------------ dones
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        # planar drive: no fall termination. tilt-based termination can be added
        # here once real rigid-body contact is modelled.
        terminated = torch.zeros_like(time_out)
        return terminated, time_out

    # ------------------------------------------------------------------ reset
    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        root_state = self.robot.data.default_root_state[env_ids].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids]
        # random initial yaw
        yaw = (torch.rand(len(env_ids), device=self.device) * 2.0 - 1.0) * torch.pi
        root_state[:, 3] = torch.cos(yaw * 0.5)   # quat w
        root_state[:, 4] = 0.0
        root_state[:, 5] = 0.0
        root_state[:, 6] = torch.sin(yaw * 0.5)   # quat z
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(len(env_ids), 6, device=self.device), env_ids)

        self._resample_commands(env_ids)
        self._resample_chaos(env_ids)
        self.prev_actions[env_ids] = 0.0
        self.actions[env_ids] = 0.0
        self._actual_twist[env_ids] = 0.0

    # ---------------------------------------------------------------- helpers
    def _yaw(self) -> torch.Tensor:
        q = self.robot.data.root_quat_w                     # (n,4) w,x,y,z
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _sample(self, env_ids, lo, hi):
        return torch.rand(len(env_ids), device=self.device) * (hi - lo) + lo

    def _resample_commands(self, env_ids):
        self.commands[env_ids, 0] = self._sample(env_ids, *self.cfg.cmd_vx_range) / self.cfg.max_lin_speed
        self.commands[env_ids, 1] = self._sample(env_ids, *self.cfg.cmd_vy_range) / self.cfg.max_lin_speed
        self.commands[env_ids, 2] = self._sample(env_ids, *self.cfg.cmd_wz_range) / self.cfg.max_yaw_rate

    def _resample_chaos(self, env_ids):
        lo, hi = self.cfg.chaos_wheel_gain
        self._wheel_gain[env_ids] = torch.rand(len(env_ids), 4, device=self.device) * (hi - lo) + lo
