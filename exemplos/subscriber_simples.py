#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose


class TurtleController(Node):
    
    def __init__(self):
        super().__init__('turtle_controller')
        self.pose = Pose(x=-1.0, y=-1.0, theta=0.0)
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )

    def pose_callback(self, msg):
        self.get_logger().info(f"O Mbappé está em x:{msg.x:.2f}, y:{msg.y:.2f}, theta:{msg.theta:.2f}")
        

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
