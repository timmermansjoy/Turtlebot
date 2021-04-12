#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan


angleList = []
avgVal = 360


class LaserListener():
    def __init__(self, output="overview"):
        self.output = output
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)

    def callback_scan(self, data):
        leftDistance = []
        rightDistance = []
        index = 0
        object_found = False
        for value in data.ranges:
            if value < 1.5:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                if current_angle < 45:
                    leftDistance.append(value)
                elif current_angle > 315:
                    rightDistance.append(value)
            index += 1

        print(len(leftDistance), len(rightDistance))

        if object_found == False:
            print("No object(s) found in range.")


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener("data")
    rospy.spin()
