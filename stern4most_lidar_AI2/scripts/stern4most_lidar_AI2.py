#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

ANGLE = 360


class LaserListener():
    def __init__(self, output="overview"):
        self.output = output
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.lidar_pub = rospy.Publisher("lidar_controller", Twist, queue_size=10)
        self.vel = Twist()
        self.vel.linear.x = 0.27
        self.rate = rospy.Rate(10)

    def publish(self, ang_vel):
        self.vel.angular.z = ang_vel
        print('publishing value: ' + str(self.vel.angular.z))
        self.lidar_pub.publish(self.vel)

    def callback_scan(self, data):
        left_weighted_values = []
        right_weighted_values = []
        index = 0
        weighted_angle = 0
        object_found = False
        for distance in data.ranges:
            if distance < 0.65:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                if current_angle < 40:
                    weighted_angle = current_angle
                    left_weighted_values.append(weighted_angle / distance)
                elif current_angle > 310:
                    weighted_angle = ANGLE - current_angle
                    right_weighted_values.append(weighted_angle / distance)
            index += 1
        avgL = sum(left_weighted_values)/len(left_weighted_values) if len(left_weighted_values) != 0 else 0
        avgR = sum(right_weighted_values)/len(right_weighted_values) if len(right_weighted_values) != 0 else 0
        print('left average: ' + str(avgL) + '  right average: ' + str(avgR))
        if avgL > avgR:
            if avgL > 0.05:
                self.publish(-0.35)
            else:
                self.publish(0)
        else:
            if avgR > 0.05:
                self.publish(0.35)
            else:
                self.publish(0)

        if object_found == False:
            print("No object(s) found in range.")


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener("data")
    rospy.spin()
