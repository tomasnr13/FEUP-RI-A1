#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt, asin
from numpy import polyfit, polyval

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
import time

WALL_DISTANCE_THRESHOLD = 2.5
FINAL_WALL_LENGTH = 3.89


MAX_ANG_VEL = 3.0
MIN_ANG_VEL = 1.5
MAX_LIN_VEL = 2.5
LOW_LIN_VEL = 1.0
class Turtle(Node):
    def __init__(self) -> None:
        super().__init__("Turtle")
        self.twist = Twist()
        self.publisher:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.min_distance_laser = (0,0)

        self.file_path = 'data.txt'
        open(self.file_path, 'w').close()

        self.laserscan = self.create_subscription(LaserScan, "/scan", self._processScan, 1)

        #random orientation
        self.twist.angular.z = random()*2*pi-pi
        self._moveRobot()
        time.sleep(3)


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
    

    def _moveRobot(self): 
        self.publisher.publish(self.twist)


    def _detectStop(self, lidar):
        # Filter out nan values
        lidar = [(angle, distance) for angle, distance in lidar if not isnan(distance)]

        # If there are not enough readings, return False
        if len(lidar) < 3:
            return False

        # Find the first, middle and last non-nan values
        first_non_nan = lidar[0]
        middle_non_nan = lidar[len(lidar) // 2]
        last_non_nan = lidar[-1]

        # Calculate the distance between the two points
        dx = first_non_nan[1] * cos(first_non_nan[0]) - last_non_nan[1] * cos(last_non_nan[0])
        dy = first_non_nan[1] * sin(first_non_nan[0]) - last_non_nan[1] * sin(last_non_nan[0])
        laser_distance = sqrt(dx**2 + dy**2)

        # Check if the distance is within the interval [3.89 +/- 0.1]
        if 3.79 <= laser_distance <= 3.99:
            if abs(first_non_nan[1]-last_non_nan[1]) < 0.1:
                # Calculate the expected distance of the middle sensor using Pythagorean theorem
                expected_middle_distance = sqrt((first_non_nan[1]**2 + last_non_nan[1]**2) / 4)
                # Check if the actual middle sensor distance is close to the expected distance
                if abs(middle_non_nan[1] - expected_middle_distance) < 0.1:
                    for angle, distance in lidar:
                        self._writeToFile(f'angle, dist: {str(angle)}, {str(distance)}')
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self._moveRobot()
                    return True

        return False


    def _reactToLidar(self, lidar):

        if self._detectStop(lidar):
            self._moveRobot()
            self.destroy_subscription(self.laserscan)
            return

        # Check for walls on the left back or right back
        left_back_wall = any(distance < WALL_DISTANCE_THRESHOLD and not isnan(distance) for angle, distance in lidar if -pi <= angle <= -pi/2)
        right_back_wall = any(distance < WALL_DISTANCE_THRESHOLD and not isnan(distance) for angle, distance in lidar if pi/2 <= angle <= pi)
        front_wall = any(distance < WALL_DISTANCE_THRESHOLD and not isnan(distance) for angle, distance in lidar if -pi/2 <= angle <= pi/2)
        
        # If a wall is detected on the left back or right back, stop moving linearly and rotate towards the wall
        if (left_back_wall or right_back_wall) and not front_wall:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -MAX_ANG_VEL*0.4 if left_back_wall else MAX_ANG_VEL*0.4
            self._moveRobot()
            return
        
        if not self._detectWall(lidar):
            self.randomWalk()
        else:
            return self._followWall(lidar)
        
    def _normalizeAsin(self, value, range):
        if value < -range:
            value = -range
        elif value > range:
            value = range
        return value / range
        
    def _followWall(self, lidar):
        angle, distance = self.min_distance_laser
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        frontLaser = lidar[len(lidar)//2]              
        frontDistance = frontLaser[1]


        if distance < WALL_DISTANCE_THRESHOLD:
            # If we are closer to the wall than desired, turn away from the wall
            self.twist.angular.z = MAX_ANG_VEL * (WALL_DISTANCE_THRESHOLD - distance)*0.6
            self.twist.linear.x = LOW_LIN_VEL
        elif distance > WALL_DISTANCE_THRESHOLD:
            # If we are farther from the wall than desired, turn towards the wall
            self.twist.angular.z = -MAX_ANG_VEL *(WALL_DISTANCE_THRESHOLD - distance)*0.6
            self.twist.linear.x = LOW_LIN_VEL

        if frontDistance != inf:
            angleRobotWall = asin(max(-1, min(distance/frontDistance, 1)))
            self.twist.angular.z += self._normalizeAsin(angleRobotWall, pi/2) * 2.0

        self.twist.linear.x = MAX_LIN_VEL

        if angle > 0: 
            self.twist.angular.z = -self.twist.angular.z
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