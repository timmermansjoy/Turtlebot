#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


angleList = []
avgVal = 360


class LaserListener():
    def __init__(self, output="overview"):
        self.output = output
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.lidarpub = rospy.Publisher("lidar_controller", Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

    def publish(self, pVel):
        self.vel.linear.z = pVel
        self.lidarpub.publish(self.vel)

    def callback_scan(self, data):
        leftDistance = []
        rightDistance = []
        index = 0
        object_found = False
        for value in data.ranges:
            if 0.5 < value < 2.5:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                if 5 < current_angle < 45:
                    leftDistance.append(value)
                elif 355 > current_angle > 315:
                    rightDistance.append(value)
            index += 1
        avgL = sum(leftDistance)/len(leftDistance) if len(leftDistance) != 0 else 0
        avgR = sum(rightDistance)/len(rightDistance) if len(rightDistance) != 0 else 0

        self.publish(avgR-avgL)

        if object_found == False:
            print("No object(s) found in range.")


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener("data")
    rospy.spin()
