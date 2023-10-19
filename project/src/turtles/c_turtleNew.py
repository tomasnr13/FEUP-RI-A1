#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from flatland_msgs.srv import MoveModel

WALL_THRESHOLD = 2
EDGE_DISTANCE = 1

def clamp(val, minVal, maxVal):
    return float(max(min(val, maxVal), minVal))

class CTurtle(Node):
    def __init__(self) -> None:
        super().__init__("CTurtle")
        self.twist = Twist()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(LaserScan, "/scan", self._processScan, 1)
        
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.file_path = 'data.txt'
        open(self.file_path, 'w').close()
        self._writeToFile('v1')

    def _writeToFile(self, line):
        with open(self.file_path, 'a') as file:
            file.write(line + '\n')

    def _processScan(self, scan):
        lidar = []
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            distance = scan.ranges[i]
            lidar.append((angle, distance))
            angle += scan.angle_increment

        self._writeToFile('processScan')
        self._writeToFile('[LIDAR]')
        self._writeToFile(f'angle min: {scan.angle_min}')
        self._writeToFile(f'angle inc: {scan.angle_increment}')
        self._writeToFile(f'angle max: {scan.angle_max}')

        self._reactToLidar(lidar)


    def _reactToLidar(self, lidar):
        #detect wall
        #if wall is not detected
        #    move randomly
        #if wall is detected
        #  move towards it frontwise until it reaches threshold
        #   and is within threshold, turn to make it move perpendicularly

        self._writeToFile('reacttoLidar')

        min_distance_laser = min(lidar, key=lambda x: x[1])
        min_angle = min_distance_laser[0]
        min_dist = min_distance_laser[1]

        self._writeToFile(f'min ang: {min_angle}')
        self._writeToFile(f'min dist: {min_dist}')

        if min_angle is not None:
            if min_angle > 0:
                twist_ang = -self.angular_speed
            elif min_angle < 0:
                twist_ang = self.angular_speed
            else:#define threshold
                twist_ang = 0

        if min_dist > WALL_THRESHOLD:
            twist_lin = self.linear_speed
        elif min_dist < WALL_THRESHOLD:
            twist_lin = -self.linear_speed
        else:
            twist_lin = 0
            twist_ang = 0
            #rotate to make perpendicular

        self.twist.linear.x = twist_lin
        self.twist.angular.z = twist_ang
        self.publisher.publish(self.twist)

def main(args = None):
    rclpy.init(args=args)
    robot = CTurtle()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()