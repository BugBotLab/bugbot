# ROS 2 integration

ROS 2 packages for BugBot. Bridges the holonomic X-omni drive into ROS 2 and runs the
corrective policy trained in [`../simulation/`](../simulation) both in sim and on the real
robot.

> Status: authored, not yet built on a live ROS 2 install. The drive mix is unit-tested;
> `colcon build` + runtime testing are pending. Target: ROS 2 Humble/Jazzy on Ubuntu / WSL 2.

## Package: `bugbot_ros` (ament_python)

```
ros2/bugbot_ros/
├── package.xml  setup.py  setup.cfg
├── bugbot_ros/
│   ├── mixing.py             X-omni mix (matches firmware + simulation) -- pure NumPy
│   ├── deploy.py             shared core: observation builder + PolicyController (sim-to-real)
│   ├── twist_bridge_node.py  /cmd_vel -> 4 wheel commands (direct teleop path)
│   └── policy_node.py        trained policy -> 4 wheel commands (learned corrective control)
├── launch/bugbot.launch.py
└── test/test_mixing_parity.py
```

### Nodes and topics

| Node | Subscribes | Publishes |
| --- | --- | --- |
| `twist_bridge` | `/cmd_vel` (`geometry_msgs/Twist`) | `/bugbot/wheel_cmd` (`std_msgs/Float32MultiArray`, BR FR BL FL) |
| `policy_node` | `/cmd_vel`, `/imu/data` (`sensor_msgs/Imu`), `/bugbot/vel` (`geometry_msgs/Twist`) | `/bugbot/wheel_cmd` |

`/cmd_vel` is a full `Twist` because BugBot is holonomic: `linear.x` = forward, `linear.y`
= left (mapped to BugBot's right-positive lateral internally), `angular.z` = yaw.

## Build and run (Ubuntu / WSL 2)

```bash
mkdir -p ~/bugbot_ws/src && cd ~/bugbot_ws/src
ln -s /path/to/bugbot/ros2/bugbot_ros .
cd ~/bugbot_ws
colcon build --packages-select bugbot_ros
source install/setup.bash

# Direct teleop path (no policy):
ros2 run bugbot_ros twist_bridge
# ... and drive it, e.g.:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.1}, angular: {z: 0.5}}"

# Learned corrective controller (zero policy until you pass one):
ros2 launch bugbot_ros bugbot.launch.py policy_path:=/path/to/policy.pt
```

The policy is the one trained in `../simulation/`, exported to TorchScript. With no
`policy_path` the controller emits a zero action (robot holds still), which is a safe
bring-up default.

## Sim-to-real

`deploy.py` holds the observation layout and normalisation constants, kept identical to the
simulation env, so a sim-trained policy consumes the same observation on the real robot.
`/bugbot/wheel_cmd` is the common output; wiring it to the firmware's command channel
(WebSocket / UDP) is the remaining integration step.
