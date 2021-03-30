#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys

def callback_controller(message):
    pub.publish(message)
    rate.sleep

if __name__ == '__main__':

    controller_topic = 'controller'
    rospy.init_node('stern4most_pilot_AI2')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    controller_sub = rospy.Subscriber(controller_topic, Twist, callback_controller)
    rate = rospy.Rate(10)

    vel = Twist()

    rospy.spin()
