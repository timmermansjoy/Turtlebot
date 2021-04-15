#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# The speed is set to a static value of 0.27. The maximum distance that will be considered 'close' is set to 1.
# Angles with a value between 0 and 40 will be considered as angles to our left.
# Angles with a value between 310 and 360 will be considered angles to our right.

SPEED = 0.27
MAX_DISTANCE = 1
MIN_ANGLE_LEFT = 0
MAX_ANGLE_LEFT = 45
MIN_ANGLE_RIGHT = 315
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

        # The for loop will iterate through all distances found in the data
        for distance in data.ranges:

            # If a distance is less than the MAX_DISTANCE, it is close enough to be considered
            if distance < MAX_DISTANCE:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                # If the corresponding angle is between MIN_ANGLE_LEFT and MAX_ANGLE_LEFT, it is considered to be an object to our left.
                if MIN_ANGLE_LEFT < current_angle < MAX_ANGLE_LEFT:

                    # To make sure that objects right in front of us get a higher weight than objects that are the side,
                    # it is necessary to subtract the current_angle from the MAX_ANGLE_LEFT. This will give an angle of 1 a bigger weight than an angle of 20.
                    weighted_angle = MAX_ANGLE_LEFT - current_angle

                    # To make sure that objects that are closer to us get a higher weight than objects that are futher away,
                    # it is necessary to divide the value in weighted_angle by the distance, 
                    # since the result of, for example,  1 / 0.5 is bigger than the result of 1 / 1.
                    left_weighted_values.append(weighted_angle / distance)

                # If the corresponding angle is between MIN_ANGLE_RIGHT and MAX_ANGLE_RIGHT, it is considered to be an object to our right.
                elif MIN_ANGLE_RIGHT < current_angle < MAX_ANGLE_RIGHT:

                    # To make sure that objects right in front of us get a higher weight than objects that are the side,
                    # it is necessary to subtract the MIN_ANGLE_RIGHT from the current_angle. This will give an angle of 360 a bigger weight than an angle of 345.
                    weighted_angle = current_angle - MIN_ANGLE_RIGHT

                    # To make sure that objects that are closer to us get a higher weight than objects that are futher away,
                    # it is necessary to divide the value in weighted_angle by the distance, 
                    # since the result of, for example,  1 / 0.5 is bigger than the result of 1 / 1.
                    right_weighted_values.append(weighted_angle / distance)
            index += 1

    
        avgL = sum(left_weighted_values)/2000 if len(left_weighted_values) > 0 else 0
        avgR = sum(right_weighted_values)/2000 if len(right_weighted_values) > 0 else 0
        print('left average: ' + str(avgL) + '  right average: ' + str(avgR))
       
        if avgL > avgR:
            if avgL > 0.05:
                self.publish(avgL * -1)
            else:
                self.publish(0)
        else:
            if avgR > 0.05:
                self.publish(avgR)
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
