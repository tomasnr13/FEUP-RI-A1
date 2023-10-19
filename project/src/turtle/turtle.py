#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

WALL_DISTANCE_THRESHOLD = 2
WALL_DISTANCE_THRESHOLD_MIN = 1.0
WALL_DISTANCE_THRESHOLD_MAX = 2.5
WALL_DISTANCE = 1
EDGE_DISTANCE = 1
MAX_ANG_VEL = 3.0
MAX_LIN_VEL = 2.0
LOW_LIN_VEL = 0.5
ANGLE_LOOSENESS = 0.1
MAX_ANGLE = 2.356194347143173
MIN_ANGLE = -2.356194347143173
ANG_INCREMENT = 0.1
LIN_VEL_DECREMENT = 0.5
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
        min_distance_laser = min(lidar, key=lambda x: x[1])
        min_dist = min_distance_laser[1]
        if min_dist is inf:
            return False
        else:
            self.min_distance_laser = min_distance_laser
            return True


    def _moveRobot(self): 
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
        

    def _reactToLidar(self, lidar):
        if not self._detectWall(lidar):
            self.randomWalk()

        else:

            min_angle, min_dist = self.min_distance_laser
            if abs((abs(min_angle) - pi/2)) < ANGLE_LOOSENESS:
                self.twist.linear.x = MAX_LIN_VEL
                self.twist.angular.z = 0.0



            elif min_dist < WALL_DISTANCE_THRESHOLD_MIN:
                if (self.twist.linear.x > LOW_LIN_VEL):
                    self.twist.linear.x -= LIN_VEL_DECREMENT

                #if(self.twist.angular.z < pi/2):
                if(self.twist.angular.z < pi):
                    self.twist.angular.z += ANG_INCREMENT

            elif min_dist > WALL_DISTANCE_THRESHOLD_MAX:
                if(min_angle < 0.0):
                    self.twist.angular.z = ANG_INCREMENT
                else:
                    self.twist.angular.z = -ANG_INCREMENT
            else:
                self.twist.linear.x = (1 - ((abs(min_angle) - MAX_ANGLE) / (MAX_ANGLE))) * MAX_LIN_VEL

                if min_angle < 0.0:
                    if min_angle < -pi/2:
                        self.twist.angular.z = -ANG_INCREMENT
                    else:
                        self.twist.angular.z = ANG_INCREMENT
                else:
                    if min_angle > pi/2:
                        self.twist.angular.z = ANG_INCREMENT
                    else:
                        self.twist.angular.z = -ANG_INCREMENT

        self._moveRobot()

        '''
            leftLaser = self._getLeftLaser(lidar)

            #robot moving parallel to wall
            if self.min_distance_laser == leftLaser:
                # Calculate the difference between the current and desired distances
                distance_error = leftLaser[1] - WALL_DISTANCE

                # Adjust angular velocity based on this difference
                self.twist.angular.z = clamp(distance_error, -MAX_ANG_VEL, MAX_ANG_VEL)

                self.twist.linear.x = MAX_LIN_VEL
                                
            #add logic to follow wall

            #robot moving forward to the wall - angle 0 (+-0.1) must be the one with less distance 
            elif (min_angle < 0.1) & (min_angle > -0.1):
                #if its too far from the wall, aproach
                if min_dist > (WALL_DISTANCE + WALL_DISTANCE_THRESHOLD):
                    
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z += MAX_ANG_VEL
                    
                #if its too close from the wall, go back
                elif min_dist < (WALL_DISTANCE - WALL_DISTANCE_THRESHOLD):
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = -MAX_ANG_VEL
                    
                else:
                    # robot is within distance, pointing to the wall
                    self.twist.angular.z = 0.0
                    #rotate clockwise to make perpendicular to wall

                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = MAX_ANG_VEL

            #robot still not aligned to wall
            elif min_angle > 0.5:
                #TODO mudar para alteração mais gradual da velocidade angular
                self.twist.angular.z = MAX_ANG_VEL
                self.twist.linear.x = 0.0
            elif min_angle < -0.5:
                #TODO mudar para alteração mais gradual da velocidade 
                self.twist.angular.z = -MAX_ANG_VEL
                self.twist.linear.x = 0.0
            self._moveRobot()
        '''

    '''
    def _reactToLidar(self, lidar):

        if not self._detectWall(lidar):
            self.randomWalk()

        else:
            #TODO: caso em que paramos
            #TODO: por a seguir a parede tbm à direita

            min_angle, min_dist = self.min_distance_laser

            leftLaser = self._getLeftLaser(lidar)

            #robot moving parallel to wall
            if self.min_distance_laser == leftLaser:
                # Calculate the difference between the current and desired distances
                distance_error = leftLaser[1] - WALL_DISTANCE

                # Adjust angular velocity based on this difference
                self.twist.angular.z = clamp(distance_error, -MAX_ANG_VEL, MAX_ANG_VEL)

                self.twist.linear.x = MAX_LIN_VEL
                                
            #add logic to follow wall

            #robot moving forward to the wall - angle 0 (+-0.1) must be the one with less distance 
            elif (min_angle < 0.1) & (min_angle > -0.1):
                #if its too far from the wall, aproach
                if min_dist > (WALL_DISTANCE + WALL_DISTANCE_THRESHOLD):
                    
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z += MAX_ANG_VEL
                    
                #if its too close from the wall, go back
                elif min_dist < (WALL_DISTANCE - WALL_DISTANCE_THRESHOLD):
                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = -MAX_ANG_VEL
                    
                else:
                    # robot is within distance, pointing to the wall
                    self.twist.angular.z = 0.0
                    #rotate clockwise to make perpendicular to wall

                    #TODO mudar para alteração mais gradual da velocidade angular
                    self.twist.angular.z = MAX_ANG_VEL

            #robot still not aligned to wall
            elif min_angle > 0.5:
                #TODO mudar para alteração mais gradual da velocidade angular
                self.twist.angular.z = MAX_ANG_VEL
                self.twist.linear.x = 0.0
            elif min_angle < -0.5:
                #TODO mudar para alteração mais gradual da velocidade 
                self.twist.angular.z = -MAX_ANG_VEL
                self.twist.linear.x = 0.0
            self._moveRobot()
    '''
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