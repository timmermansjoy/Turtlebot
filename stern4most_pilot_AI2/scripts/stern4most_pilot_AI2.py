#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys

is_manual = [True]
def control_turtlebot(message=None):
    if is_manual[0] and message:
        move_turtle_bot(message)
    else:
        print('moving turtlebot with autotwist message')
        move_turtle_bot(autotwist)
        rate.sleep()

def move_turtle_bot(message):
    pub.publish(message)

def set_manual_autonomous(msg):
    print("received Bool " + str(msg) + " with data " + str(msg.data))
    is_manual[0] = msg.data
    control_turtlebot()

if __name__ == '__main__':

    controller_topic = 'controller'
    manual_autonomous_topic = 'manual_autonomous'
    rospy.init_node('stern4most_pilot_AI2')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    controller_sub = rospy.Subscriber(controller_topic, Twist, control_turtlebot)
    man_auto_sub = rospy.Subscriber(manual_autonomous_topic, Bool, set_manual_autonomous)
    autotwist = Twist()
    autotwist.linear.x = 0.22
    rate = rospy.Rate(10)

    vel = Twist()

    rospy.spin()
