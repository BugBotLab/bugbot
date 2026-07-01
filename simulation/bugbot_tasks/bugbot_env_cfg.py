"""DirectRLEnv config for the BugBot holonomic velocity-tracking task.

The task: track a commanded body twist (vx, vy, wz) despite a deliberately
chaotic X-omni drive. The policy outputs four wheel commands; the env turns
those into an *actual* body motion through the firmware mix plus a per-env chaos
model (gain/slip, translation<->rotation coupling, vibration, latency), and the
policy must learn the micro-corrections that keep BugBot on the commanded twist.

Authored for Isaac Lab v2.3.2; validate/tune on a real Isaac Lab run.
"""

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .bugbot_cfg import BUGBOT_CFG


@configclass
class BugBotEnvCfg(DirectRLEnvCfg):
    # --- run control ---
    decimation = 4
    episode_length_s = 10.0

    # --- spaces ---
    # action: 4 wheel commands in [-1, 1] (BR, FR, BL, FL)
    action_space = 4
    # obs: command twist (3) + measured lin vel xy (2) + measured yaw rate (1)
    #      + previous action (4) = 10
    observation_space = 10
    state_space = 0

    # --- sim ---
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # --- scene (parallel envs) ---
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=2.0, replicate_physics=True)

    # --- robot ---
    robot: object = BUGBOT_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # --- drive limits (map normalised wheel/twist to physical units) ---
    max_lin_speed = 0.5      # m/s at |twist| = 1
    max_yaw_rate = 3.0       # rad/s at |rot| = 1
    wheel_spin_scale = 40.0  # rad/s visual wheel spin per unit wheel command

    # --- command sampling ranges (body twist to track) ---
    cmd_vx_range = (-0.4, 0.4)
    cmd_vy_range = (-0.3, 0.3)
    cmd_wz_range = (-2.0, 2.0)

    # --- chaos model (per-env, resampled on reset). These are the "hard part". ---
    chaos_wheel_gain = (0.7, 1.3)      # per-wheel multiplicative gain (slip/variation)
    chaos_coupling = 0.25              # fraction of commanded motion that leaks across axes
    chaos_vibration_std = 0.06         # additive white noise on the actual twist
    chaos_latency = 0.4                # first-order lag coefficient (0 = none, ->1 = sluggish)

    # --- sensor noise (observations the policy sees) ---
    obs_linvel_noise_std = 0.02
    obs_yawrate_noise_std = 0.03

    # --- reward weights ---
    rew_track_lin = 1.5
    rew_track_yaw = 0.75
    rew_effort = -0.002
    rew_action_rate = -0.01
    rew_tracking_sigma = 0.25          # width of the exp() tracking reward


@configclass
class BugBotEnvCfg_PLAY(BugBotEnvCfg):
    def __post_init__(self):
        self.scene.num_envs = 32
        self.scene.env_spacing = 1.0
        # deterministic-ish for viewing
        self.chaos_vibration_std = 0.03
