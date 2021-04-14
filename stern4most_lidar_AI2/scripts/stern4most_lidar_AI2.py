#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

ANGLE = 360


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
        self.vel.linear.x = 0.27
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
