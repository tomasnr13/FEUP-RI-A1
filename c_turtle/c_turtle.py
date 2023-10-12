#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from flatland_msgs.srv import MoveModel


def clamp(val, minVal, maxVal):
    return float(max(min(val, maxVal), minVal))


def pointDist(p1, p2) -> float:
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def laserToPoint(laser):
    return (laser[1] * cos(laser[0]), laser[1] * sin(laser[0]))

class CTurtle(Node):
    def __init__(self) -> None:
        super().__init__("CTurtle")
        # self.vel = Twist()
        self.create_subscription(LaserScan, "/scan", self._scanCallback, 1)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)

    def _scanCallback(self, msg):
        # Assume the laser scan provides range data in the front direction
        front_distance = msg.ranges[len(msg.ranges)//2]

        # Set a threshold for detecting a wall (adjust as needed)
        wall_threshold = 1.0

        # If the distance to the wall is less than the threshold, approach the wall
        if front_distance < wall_threshold:
            self.approach_wall()

    def approach_wall(self):
        # Publish a Twist message to move the robot forward
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Adjust linear velocity as needed
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

def main(args = None):
    rclpy.init(args=args)
    robot = CTurtle()
    rclpy.spin(robot)
    rclpy.shutdown()

if __name__ == "__main__":
    main()