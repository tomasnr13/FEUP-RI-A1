#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

WALL_DISTANCE_THRESHOLD = 2.5

MAX_ANG_VEL = 3.0
MAX_LIN_VEL = 2.0
LOW_LIN_VEL = 1.5
class Turtle(Node):
    def __init__(self) -> None:
        super().__init__("Turtle")
        self.twist = Twist()
        self.publisher:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.min_distance_laser = (0,0)

        self.file_path = 'data.txt'
        open(self.file_path, 'w').close()

        self.create_subscription(LaserScan, "/scan", self._processScan, 1)

    def _writeToFile(self, line):
        with open(self.file_path, 'a') as file:
            file.write(line + '\n')

    def _processScan(self, scan):
        lidar = []
        angle = scan.angle_min
        self._writeToFile('lasers')
        for i in range(len(scan.ranges)):
            distance = scan.ranges[i]
            lidar.append((angle, distance))
            self._writeToFile(f'angle, dist: {str(angle)}, {str(distance)}')
            angle += scan.angle_increment

        self._reactToLidar(lidar)

    def _detectWall(self, lidar):
        # replace nans with inf
        lidar = [(angle, inf) if isnan(distance) else (angle, distance) for angle, distance in lidar]
        
        # Filter out lidar readings that are outside the range [-pi/2, pi/2]
        lidar = [(angle, distance) for angle, distance in lidar if -pi/2 <= angle <= pi/2]
        
        min_distance_laser = min(lidar, key=lambda x: x[1])
        min_dist = min_distance_laser[1]
        if min_dist is inf:
            return False
        else:
            self.min_distance_laser = min_distance_laser
            return True

    def _detectEdges(self, lidar):
        firstVertex = False
        firstVertexLaser = ()
        edge = False

        for i in range(0, len(lidar)):
            if lidar[i][1] == inf & (not firstVertex):
                firstVertex = True
            elif lidar[i][1] == inf & firstVertex & (not edge):
                firstVertex = False
            elif lidar[i][1] != inf & firstVertex & (not edge):
                edge = True
                firstVertexLaser = lidar[i]
            elif lidar[i][1] == inf & firstVertex & edge:
                return (firstVertexLaser,lidar[i-1])
        return ()
    
    def _calculateDistLasers(self, laser1, laser2):
        return sqrt(laser1[1]**2 + laser2[1]**2)

    def _moveRobot(self): 
        self.publisher.publish(self.twist)

    def _reactToLidar(self, lidar):
        if not self._detectWall(lidar):
            self.randomWalk()
        else:
            return self._followWall(lidar)
        
    def _followWall(self, lidar):
        angle, distance = self.min_distance_laser
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        # Check if there's a wall directly in front of the robot
        front_distance = min(dist for ang, dist in lidar if abs(ang) < pi/8)
        if front_distance < WALL_DISTANCE_THRESHOLD:
            # Wall in front, turn right and move slowly
            self.twist.linear.x = LOW_LIN_VEL
            self.twist.angular.z = (WALL_DISTANCE_THRESHOLD - distance)
        elif distance < WALL_DISTANCE_THRESHOLD:
            # Too close to the wall, turn right
            self.twist.linear.x = LOW_LIN_VEL
            self.twist.angular.z = (WALL_DISTANCE_THRESHOLD - distance)
        elif distance > WALL_DISTANCE_THRESHOLD:
            # Too far from the wall, turn left
            self.twist.linear.x = LOW_LIN_VEL
            self.twist.angular.z = -(distance - WALL_DISTANCE_THRESHOLD)
        else:
            # Ideal distance to the wall, move forward
            self.twist.linear.x = MAX_LIN_VEL

        self._moveRobot()



    #wiggle
    def randomWalk(self):
        v = MAX_ANG_VEL * random()
        if random() > 0.5:
            self.twist.angular.z += v
        else:
            self.twist.angular.z -= v
        # reset wandering angle when limit reached
        if abs(self.twist.angular.z) >= MAX_ANG_VEL:
            self.twist.angular.z = 0.0

        self.twist.linear.x = MAX_LIN_VEL
        self._moveRobot()


def main(args = None):
    rclpy.init(args=args)
    robot = Turtle()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()