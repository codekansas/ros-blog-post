import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtlesimController(Node):
    def __init__(self, run_every_n_seconds: float = 0.5, queue_size: int = 10) -> None:
        """Defines a simple controller for Turtlesim.

        Args:
            run_every_n_seconds: Run the publisher every N seconds
            queue_size: The number of messages to queue up if the subscriber
                is not receiving them fast enough; this is a quality-of-service
                setting in ROS
        """

        super().__init__(node_name="turtle_sim_controller")

        self.turtle_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=queue_size)
        self.timer = self.create_timer(run_every_n_seconds, self.timer_callback)

    def timer_callback(self) -> None:
        """Defines a callback which is called every time the timer spins."""

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = math.pi
        self.turtle_pub.publish(msg)
        self.get_logger().info("Published a message")


def run_turtlesim_controller(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)

    controller = TurtlesimController()

    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run_turtlesim_controller()
