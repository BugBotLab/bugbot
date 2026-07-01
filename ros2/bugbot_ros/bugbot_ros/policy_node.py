"""Learned corrective controller for BugBot.

Runs a policy trained in the Isaac Lab simulation (exported to TorchScript) that
maps [command twist + measured motion + previous action] to four wheel commands,
applying the micro-corrections that counter the chaotic X-omni drive. Publishes
``/bugbot/wheel_cmd`` (BR, FR, BL, FL) at a fixed rate.

Inputs:
  /cmd_vel        geometry_msgs/Twist   desired body twist (linear.x, linear.y, angular.z)
  /imu/data       sensor_msgs/Imu       measured yaw rate (angular_velocity.z)
  /bugbot/vel     geometry_msgs/Twist   measured body linear velocity (vx, vy) [optional]

Parameters:
  policy_path (str)   TorchScript policy; empty = zero policy (holds still), safe for bring-up
  control_hz (float)  control loop rate (default 50)

    ros2 run bugbot_ros policy_node --ros-args -p policy_path:=/path/to/policy.pt

Frame note: ROS y is left+, BugBot lateral is right+, so lateral = -linear.y.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from .deploy import PolicyController, RobotSensors


class PolicyNode(Node):
    def __init__(self):
        super().__init__("bugbot_policy")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("control_hz", 50.0)
        policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        control_hz = self.get_parameter("control_hz").get_parameter_value().double_value

        self._controller = PolicyController(policy_path or None)
        self._cmd = np.zeros(3, dtype=np.float32)     # (vx, vy_right, wz) SI
        self._sensors = RobotSensors()

        self._pub = self.create_publisher(Float32MultiArray, "/bugbot/wheel_cmd", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.create_subscription(Twist, "/bugbot/vel", self._on_vel, 10)
        self.create_timer(1.0 / max(control_hz, 1.0), self._on_tick)

        mode = "zero policy (bring-up)" if not policy_path else policy_path
        self.get_logger().info(f"bugbot_policy up @ {control_hz:.0f} Hz, policy = {mode}")

    def _on_cmd_vel(self, msg: Twist):
        self._cmd[:] = (msg.linear.x, -msg.linear.y, msg.angular.z)

    def _on_imu(self, msg: Imu):
        self._sensors.yaw_rate = float(msg.angular_velocity.z)

    def _on_vel(self, msg: Twist):
        self._sensors.lin_vel_xy = np.array([msg.linear.x, -msg.linear.y], dtype=np.float64)

    def _on_tick(self):
        wheels = self._controller.act(self._cmd, self._sensors)
        out = Float32MultiArray()
        out.data = [float(w) for w in wheels]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
