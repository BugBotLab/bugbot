"""BugBot Isaac Lab task (holonomic X-omni velocity tracking).

Importing this package registers the Gym task ids. The provided train.py / play.py
put the ``simulation`` directory on sys.path so ``import bugbot_tasks`` works.
"""

import gymnasium as gym

gym.register(
    id="Isaac-BugBot-Track-Direct-v0",
    entry_point="bugbot_tasks.bugbot_env:BugBotEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bugbot_env_cfg:BugBotEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:BugBotPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-BugBot-Track-Direct-Play-v0",
    entry_point="bugbot_tasks.bugbot_env:BugBotEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bugbot_env_cfg:BugBotEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:BugBotPPORunnerCfg",
    },
)
