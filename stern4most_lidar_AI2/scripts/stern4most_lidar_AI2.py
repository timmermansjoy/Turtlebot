#!/usr/bin/env python3

import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

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

        self.sternformost_sub = rospy.Subscriber("sternformost", Bool, self.callback_sternformost)
        rospy.loginfo("subscribed to topic sternformost")

        self.BACKWARDS = Bool()
        self.BACKWARDS.data = False

        # The speed is set to a static value of 0.27. The maximum distance that will be considered 'close' is set to 1.
        # Angles with a value between 0 and 40 will be considered as angles to our left.
        # Angles with a value between 310 and 360 will be considered angles to our right.
        self.SPEED = -0.15 if self.BACKWARDS else 0.27
        self.MAX_DISTANCE = 2.5 if self.BACKWARDS else 1.0
        self.MIN_ANGLE_LEFT = 135 if self.BACKWARDS else 0
        self.MAX_ANGLE_LEFT = 180 if self.BACKWARDS else 45
        self.MIN_ANGLE_RIGHT = 180 if self.BACKWARDS else 315
        self.MAX_ANGLE_RIGHT = 225 if self.BACKWARDS else 360

        # The publisher for the lidar_controller topic will publish Twist messages with a static linear.x value of 0.27.
        # The rate is set to 10.
        self.vel = Twist()
        self.vel.linear.x = self.SPEED
        self.rate = rospy.Rate(10)

    def callback_scan(self, data):
        """
        This method determines whether or not an object has been detected, and if so, which side the turtlebot should turn to.
        """

        # The weighted values are stored in arrays.
        left_weighted_values = []
        right_weighted_values = []
        all_angles = []
        index = 0
        weighted_angle = 0
        object_found = False

        # The for loop will iterate through all distances found in the data
        for distance in data.ranges:
            all_angles.append(distance)
            # If a distance is less than the MAX_DISTANCE, it is close enough to be considered
            if distance < self.MAX_DISTANCE:
                if object_found == False:
                    object_found = True
                # look at every degree around the robot
                current_angle = data.angle_min + (data.angle_increment * index)
                current_angle = math.degrees(current_angle)

                # If the corresponding angle is between MIN_ANGLE_LEFT and MAX_ANGLE_LEFT, it is considered to be an object to our left.
                if self.MIN_ANGLE_LEFT < current_angle < self.MAX_ANGLE_LEFT:

                    # To make sure that objects right in front of us get a higher weight than objects that are the side,
                    # it is necessary to subtract the current_angle from the MAX_ANGLE_LEFT. This will give an angle of 1 a bigger weight than an angle of 20.
                    weighted_angle = self.MAX_ANGLE_LEFT - current_angle

                    # To make sure that objects that are closer to us get a higher weight than objects that are futher away,
                    # it is necessary to divide the value in weighted_angle by the distance,
                    # since the result of, for example,  1 / 0.5 is bigger than the result of 1 / 1.
                    left_weighted_values.append(weighted_angle / distance)

                # If the corresponding angle is between MIN_ANGLE_RIGHT and MAX_ANGLE_RIGHT, it is considered to be an object to our right.
                elif self.MIN_ANGLE_RIGHT < current_angle < self.MAX_ANGLE_RIGHT:

                    # To make sure that objects right in front of us get a higher weight than objects that are the side,
                    # it is necessary to subtract the MIN_ANGLE_RIGHT from the current_angle. This will give an angle of 360 a bigger weight than an angle of 345.
                    weighted_angle = current_angle - self.MIN_ANGLE_RIGHT

                    # To make sure that objects that are closer to us get a higher weight than objects that are futher away,
                    # it is necessary to divide the value in weighted_angle by the distance,
                    # since the result of, for example,  1 / 0.5 is bigger than the result of 1 / 1.
                    right_weighted_values.append(weighted_angle / distance)
            index += 1
        print(len(all_angles))
        if object_found == True:

            # To determine whether the turtlebot should turn left or right to avoid the found object, we calculate the average of the left_weighted_values and right_weighted_values arrays.
            # This is done by taking the sum of the arrays and dividing this by 3000.
            # We chose to go with 3000 because the array can contain 40 values at most, but this is hardly ever the case, so to make up for that we will divide it by 30.
            # To then get a number that is usable as the angular.z value for the Twist message, it should be divided by 100, thus making 3000.
            avgL = sum(left_weighted_values)/3000 if len(left_weighted_values) > 0 else 0
            avgR = sum(right_weighted_values)/3000 if len(right_weighted_values) > 0 else 0
            rospy.loginfo('left average: ' + str(avgL) + '  right average: ' + str(avgR))

            # If the avgL has a greater value than the avgR, we should turn right.
            if avgL > avgR:

                # Values below 0.05 are not published, giving the vision more chance to correct our position on the track. If the value is smaller than 0.05, the node will publish a value of 0.
                # If the value is greater than 0.05, it should be multiplied by -1, since turning right is indicated by a negative angular.z value.
                if avgL > 0.05:
                    self.publish(avgL * -1)
                else:
                    self.publish(0)

            # If the avgR has a greater value than the avgL, we should turn left.
            else:

                # The value that is published should not be multiplied by -1 in this case, since turning left is indicated by a positive angular.z value.
                if avgR > 0.05:
                    self.publish(avgR)
                else:
                    self.publish(0)

        else:
            print("No object(s) found in range.")
    
    def callback_sternformost(self, data):
        self.BACKWARDS = data

    def publish(self, ang_vel):
        """
        This method takes an int, sets it as the angular.z value of the Twist message that was declared at the top of this file, and publishes it to the lidar_controller topic.
        """
        self.vel.angular.z = ang_vel
        rospy.loginfo('publishing value: ' + str(self.vel.angular.z))
        self.lidar_pub.publish(self.vel)


if __name__ == "__main__":
    rospy.init_node("laser_listener")
    LaserListener()
    rospy.spin()
