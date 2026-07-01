"""Isaac Lab articulation config for the BugBot X-omni robot.

Spawns the USD converted from ``robot/bugbot.urdf`` (see README for the
convert_urdf step) and defines velocity-controlled wheel actuators.

NOTE: authored for Isaac Lab v2.3.2 and NOT yet validated on hardware/sim.
Expect to tune masses, actuator gains, and the spawn height once run.
"""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# Produced by: python IsaacLab/scripts/tools/convert_urdf.py \
#   simulation/robot/bugbot.urdf simulation/robot/usd/bugbot.usd --headless
_USD_PATH = str(Path(__file__).resolve().parents[1] / "robot" / "usd" / "bugbot.usd")

WHEEL_JOINTS = ["joint_wheel_br", "joint_wheel_fr", "joint_wheel_bl", "joint_wheel_fl"]

BUGBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_USD_PATH,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.035),   # chassis half-height + wheel radius; tune once run
        joint_vel={".*": 0.0},
    ),
    actuators={
        # Wheels are velocity-driven. Stiffness 0 + damping so velocity targets track.
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["joint_wheel_.*"],
            effort_limit=2.0,
            velocity_limit=60.0,
            stiffness=0.0,
            damping=0.5,
        ),
    },
)
