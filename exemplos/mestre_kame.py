#! /usr/bin/env python3

import csv

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from collections import namedtuple, deque

MAX_DIFF = 0.1
VEL = 0.5


class MapReader():

    def __init__(self, map_file="./map.csv"):
        self.map_file = map_file

    def read_map(self):
        pose_queue = deque()
        with open(self.map_file) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:
                new_pose = Pose(x=0.0, y=0.0, theta=0.0)
                new_pose.x, new_pose.y = [float(x) for x in row]
                pose_queue.append(new_pose)
        return pose_queue


class TurtleController(Node):

    def __init__(self, name="meste_kame"):
        super().__init__(node_name=name)
        self._cmd_vel_publisher = self.create_publisher(
                msg_type=Twist,
                topic="/turtle1/cmd_vel",
                qos_profile=10)
        self._pose_listener = self.create_subscription(
                msg_type=Pose,
                topic="/turtle1/pose",
                callback=self.pose_callback,
                qos_profile=10)
        self.pose_queue = MapReader().read_map()
        self.pose = None
        self.initial_pos = None
        self.setpoint = Pose(x=0.0, y=0.0, theta=0.0)

    def pose_callback(self, msg):
        if self.pose is None:
            self.pose = Pose(x=msg.x, y=msg.y, theta=0.0)
            self.initial_pos = self.pose
            self.change_setpoint()
        else:
            self.pose.x = msg.x
            self.pose.y = msg.y
        self.control_turtle()

    def change_setpoint(self):
        if not self.pose_queue:
            self.get_logger().info("Acabei! =)")
            self.destroy_node()
            return
        new_setpoint = self.pose_queue.popleft()
        self.initial_pos = self.pose
        self.setpoint.x = new_setpoint.x + self.initial_pos.x
        self.setpoint.y = new_setpoint.y + self.initial_pos.y

    def control_turtle(self):
        diff_x = self.setpoint.x - self.pose.x
        diff_y = self.setpoint.y - self.pose.y
        if abs(diff_x) < MAX_DIFF and abs(diff_y) < MAX_DIFF:
            self.change_setpoint()
            return
        self.get_logger().info(f"diff_x: {diff_x:.2f}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0
        if diff_x > MAX_DIFF:
            cmd_vel_msg.linear.x = VEL
        elif diff_x < -MAX_DIFF:
            cmd_vel_msg.linear.x = -VEL
        else:
            cmd_vel_msg.linear.x = 0.0
        if diff_y > MAX_DIFF:
            cmd_vel_msg.linear.y = VEL
        elif diff_y < -MAX_DIFF:
            cmd_vel_msg.linear.y = -VEL
        else:
            cmd_vel_msg.linear.y = 0.0
        self._cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
