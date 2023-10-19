#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

WALL_DISTANCE_THRESHOLD = 2
WALL_DISTANCE = 0.5
EDGE_DISTANCE = 1
MAX_ANG_VEL = 3.0
MAX_LIN_VEL = 2.0

def clamp(val, minVal, maxVal):
    return float(max(min(val, maxVal), minVal))

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
        min_distance_laser = min(lidar, key=lambda x: x[1])
        min_dist = min_distance_laser[1]
        if not isnan(min_dist):
            self.min_distance_laser = min_distance_laser
            return True
        else:
            return False

    def _moveRobot(self): #self, twist_lin, twist_ang):
        #self.twist.linear.x = twist_lin
        #self.twist.angular.z = twist_ang
        #self._writeToFile('[MOVE]')
        #self._writeToFile(f'Linear: {twist_lin}')
        #self._writeToFile(f'Angular: {twist_ang}')
        self.publisher.publish(self.twist)

    def _getLeftLaser(self, lidar):
        closest_pair = None
        min_angle_difference = float('inf')

        for angle, distance in lidar:
            angle_difference = abs(angle + pi / 2)

            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                closest_pair = (angle, distance)

        return closest_pair


    def _reactToLidar(self, lidar):
        #detect wall
        #if wall is not detected
        #    move randomly
        #if wall is detected
        #  move towards it frontwise until it reaches threshold
        #  when robot is within wall threshold, rotate 90 degrees so he is parallel to the wall, with the wall on his left side
        #  move forward and adapt angular velocity so the left laser distance keeps the same values
        #  if laser distance increases, rotate counterclockwise and vice versa

        wall_detected = self._detectWall(lidar)

        if wall_detected:

            #TODO: caso em que paramos
            #TODO: por a seguir a parede tbm à direita

            min_angle = self.min_distance_laser[0]
            min_dist = self.min_distance_laser[1]

            leftLaser = self._getLeftLaser(lidar)

            #robot moving parallel to wall
            if self.min_distance_laser == leftLaser:
                self.twist.angular.z = 0

                #TODO mudar para alteração mais gradual da velocidade 
                self.twist.linear.x = MAX_LIN_VEL
                                
            #add logic to follow wall

            #robot moving forward to the wall - angle 0 (+-0.1) must be the one with less distance 
            elif (min_angle < 0.1) & (min_angle > -0.1):
                twist_ang = 0
                #if its too far from the wall, aproach
                if min_dist > (WALL_DISTANCE + WALL_DISTANCE_THRESHOLD):
                    
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = MAX_ANG_VEL
                    
                #if its too close from the wall, go back
                elif min_dist < (WALL_DISTANCE - WALL_DISTANCE_THRESHOLD):
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = -MAX_ANG_VEL
                    
                else:
                    # robot is within distance, pointing to the wall
                    self.twist.angular.z = 0
                    #rotate clockwise to make perpendicular to wall

                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = MAX_ANG_VEL

            #robot still not aligned to wall
            elif min_angle > 0.5:
                #TODO mudar para alteração mais gradual da velocidade angular
                self.twist.angular.z = MAX_ANG_VEL
                self.twist.linear.x = 0
            elif min_angle < -0.5:
                #TODO mudar para alteração mais gradual da velocidade 
                self.twist.angular.z = -MAX_ANG_VEL
                self.twist.linear.x = 0

            self._moveRobot(twist_lin, twist_ang)
        else:
            self.randomWalk()

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