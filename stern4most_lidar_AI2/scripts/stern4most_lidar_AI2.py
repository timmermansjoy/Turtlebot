#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SPEED = 0.27
MAX_DISTANCE = 0.65
MIN_ANGLE_LEFT = 0
MAX_ANGLE_LEFT = 40
MIN_ANGLE_RIGHT = 310
MAX_ANGLE_RIGHT = 360


class LaserListener():
    def __init__(self):
        # To get the data from the laser, the lidar node has to subscribe to the /scan topic which will return a LaserScan object. 
        # The incoming messages will be handled by the callback_scan method.
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.loginfo("Subscribed to topic /scan")

        # To send the info, calculated in this class, back to the pilot node, the lidar node has to have a publisher for the lidar_controller.
        # It will publish a Twist message, with the queue_size set to 10.
        self.lidar_pub = rospy.Publisher("lidar_controller", Twist, queue_size=10)
        rospy.loginfo("Created publisher for topic lidar_controller")

        # The publisher for the lidar_controller topic will publish Twist messages with a static linear.x value of 0.27. 
        # The rate is set to 10.
        self.vel = Twist()
        self.vel.linear.x = SPEED
        self.rate = rospy.Rate(10)

    def callback_scan(self, data):
        """
        This method determines whether or not an object has been detected, and if so, which side the turtlebot should turn to.
        """

        # The weighted values are stored in arrays.
        left_weighted_values = []
        right_weighted_values = []
        index = 0
        weighted_angle = 0
        object_found = False
        for distance in data.ranges:
            if distance < MAX_DISTANCE:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                if MIN_ANGLE_LEFT < current_angle < MAX_ANGLE_LEFT:
                    weighted_angle = MAX_ANGLE_LEFT - current_angle
                    left_weighted_values.append(weighted_angle / distance)
                elif MIN_ANGLE_RIGHT < current_angle < MAX_ANGLE_RIGHT:
                    weighted_angle = current_angle - MIN_ANGLE_RIGHT
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
    
    def publish(self, ang_vel):
        """
        
        """

        self.vel.angular.z = ang_vel
        rospy.loginfo('publishing value: ' + str(self.vel.angular.z))
        self.lidar_pub.publish(self.vel)


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener()
    rospy.spin()
