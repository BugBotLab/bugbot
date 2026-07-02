"""Direct /cmd_vel -> wheel-command bridge (no policy).

Subscribes to a standard ``geometry_msgs/Twist`` on ``/cmd_vel`` and publishes
the four normalised wheel commands (BR, FR, BL, FL) on
``/bugbot/wheel_cmd`` (``std_msgs/Float32MultiArray``) using the firmware X-omni
mix. This is the teleop / open-loop path; the learned corrective controller is
``policy_node``.

Frame note: ROS uses y-left-positive (REP-103); BugBot's lateral is right-
positive, so lateral = -linear.y.

    ros2 run bugbot_ros twist_bridge
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .deploy import twist_to_wheel_cmds


class TwistBridge(Node):
    def __init__(self):
        super().__init__("bugbot_twist_bridge")
        self._pub = self.create_publisher(Float32MultiArray, "/bugbot/wheel_cmd", 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.get_logger().info("bugbot_twist_bridge up: /cmd_vel -> /bugbot/wheel_cmd (BR,FR,BL,FL)")

    def _on_cmd_vel(self, msg: Twist):
        wheels = twist_to_wheel_cmds(
            vx=msg.linear.x,
            vy_right=-msg.linear.y,   # ROS y is left+, BugBot lateral is right+
            wz=msg.angular.z,
        )
        out = Float32MultiArray()
        out.data = [float(w) for w in wheels]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
