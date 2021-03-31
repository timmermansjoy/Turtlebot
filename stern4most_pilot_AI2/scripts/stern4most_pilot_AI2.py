#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import logging
import sys

def callback_controller(message):
    logging.info('received message from topic controller')
    logging.info('advertising to topic /cmd_vel with linear x value of ' + str(message.linear.x) + ' and angular z value of ' + str(message.angular.z))
    pub.publish(message)
    rate.sleep

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    controller_topic = 'controller'
    rospy.init_node('stern4most_pilot_AI2')
    logging.info('node stern4most_pilot_AI2 has been initialized')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    logging.info('created publisher for topic /cmd_vel')
    controller_sub = rospy.Subscriber(controller_topic, Twist, callback_controller)
    logging.info('subscribed to topic controller')
    rate = rospy.Rate(10)
    vel = Twist()

    rospy.spin()
