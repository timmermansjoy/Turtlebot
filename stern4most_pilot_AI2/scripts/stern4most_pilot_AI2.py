#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import logging
import sys
from std_msgs.msg import Bool, String

is_autonomous = [False]


def callback_autonomous_controller(message):
    if is_autonomous[0]:
        rospy.loginfo('publishing to pilot from autonomous_controller')
        publish_to_pilot(message)
    else:
        pass


def callback_manual_controller(message):
    if not is_autonomous[0]:
        rospy.loginfo('publishing to pilot from manual_controller')
        publish_to_pilot(message)
    else:
        pass


def publish_to_pilot(message):
    rospy.loginfo('advertising to topic /cmd_vel with linear x value of ' + str(message.linear.x) + ' and angular z value of ' + str(message.angular.z))
    pub.publish(message)
    rate.sleep


def callback_manual_autonomous(msg):
    rospy.loginfo('changing is_autonomous from ' + str(is_autonomous[0]) + ' to ' + str(msg.data))
    is_autonomous[0] = msg.data
    if not is_autonomous[0]:
        publish_to_pilot(stop_vel)
    rospy.loginfo('new val is_autonomous: ' + str(is_autonomous[0]))

# def callback_lidar_controller(msg):


if __name__ == '__main__':
    rospy.init_node('stern4most_pilot_AI2')
    rospy.loginfo('node stern4most_pilot_AI2 has been initialized')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo('created publisher for topic /cmd_vel')
    manual_controller_sub = rospy.Subscriber('manual_controller', Twist, callback_manual_controller)
    rospy.loginfo('subscribed to topic manual_controller')
    autonomous_controller_sub = rospy.Subscriber('autonomous_controller', Twist, callback_autonomous_controller)
    rospy.loginfo('subscribed to topic autonomous_controller')
    manual_autonomous_sub = rospy.Subscriber('manual_autonomous', Bool, callback_manual_autonomous)
    rospy.loginfo('subscribed to topic manual_autonomous')
    #lidar_controller_sub = rospy.Subscriber('lidar_controller', Twist, callback_lidar_controller)
    rate = rospy.Rate(10)
    stop_vel = Twist()

    rospy.spin()
