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
        index = 0
        object_found = False
        for value in data.ranges:
            if value < 1.5:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)
                angleList.append(current_angle)
                if len(angleList) > avgVal:
                    angleList.pop(0)
                leftValues = [x for x in angleList if x < 45]
                rightValues = [x for x in angleList if x > 315]

                leftTurnCurve = len(leftValues) / 166
                rightTurnCurve = len(rightValues) / 166
                # print("left", leftValues)
                # print("right", rightValues)
                angle = sum(angleList)//len(angleList)
            index += 1
        print(len(leftValues), len(rightValues))
        print(leftTurnCurve, rightTurnCurve)

        if object_found == False:
            print("No object(s) found in range.")


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener("data")
    rospy.spin()
