"""Roll out / view a trained BugBot policy in Isaac Lab.

    C:\\Users\\Jerome\\isaac\\.venv\\Scripts\\python.exe -m bugbot_tasks.play --checkpoint bugbot_tasks/runs/bugbot_track/<ts>/model_<n>.pt --num_envs 32
"""

import argparse
import os
import sys

import torch  # noqa: F401,E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a trained BugBot policy.")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=32)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import bugbot_tasks  # noqa: F401,E402
from bugbot_tasks.bugbot_env_cfg import BugBotEnvCfg_PLAY  # noqa: E402
from bugbot_tasks.rsl_rl_ppo_cfg import BugBotPPORunnerCfg  # noqa: E402

TASK = "Isaac-BugBot-Track-Direct-Play-v0"


def main():
    env_cfg = BugBotEnvCfg_PLAY()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = gym.make(TASK, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env)

    runner = OnPolicyRunner(env, BugBotPPORunnerCfg().to_dict(), log_dir=None, device="cuda:0")
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.get_observations()
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
