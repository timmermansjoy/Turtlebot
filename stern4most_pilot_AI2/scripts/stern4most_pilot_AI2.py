#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import logging
import sys
from std_msgs.msg import Bool, String


class Pilot():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo('created publisher for topic /cmd_vel')
        self.manual_controller_sub = rospy.Subscriber('manual_controller', Twist, self.callback_manual_controller)
        rospy.loginfo('subscribed to topic manual_controller')
        self.autonomous_controller_sub = rospy.Subscriber('autonomous_controller', Twist, self.callback_autonomous_controller)
        rospy.loginfo('subscribed to topic autonomous_controller')
        self.manual_autonomous_sub = rospy.Subscriber('manual_autonomous', Bool, self.callback_manual_autonomous)
        rospy.loginfo('subscribed to topic manual_autonomous')
        self.lidar_controller_sub = rospy.Subscriber('lidar_controller', Twist, self.callback_lidar_controller)
        rospy.loginfo('subscribed to topic lidar_controller')
        self.rate = rospy.Rate(10)
        self.stop_vel = Twist()
        self.is_autonomous = False
        self.object_detected = False
        #self.lidar_message = Twist()

    def callback_autonomous_controller(self, msg):
        if self.is_autonomous and not self.object_detected:
            rospy.loginfo('publishing to pilot from autonomous_controller')
            self.publish_to_pilot(msg)

    def callback_manual_controller(self, msg):
        if not self.is_autonomous:
            rospy.loginfo('publishing to pilot from manual_controller')
            self.publish_to_pilot(msg)

    def publish_to_pilot(self, msg):
        rospy.loginfo('advertising to topic /cmd_vel with linear x value of ' + str(msg.linear.x) + ' and angular z value of ' + str(msg.angular.z))
        self.pub.publish(msg)
        self.rate.sleep

    def callback_manual_autonomous(self, msg):
        rospy.loginfo('changing is_autonomous from ' + str(self.is_autonomous) + ' to ' + str(msg.data))
        self.is_autonomous = msg.data
        if not self.is_autonomous:
            self.publish_to_pilot(self.stop_vel)
        rospy.loginfo('new val is_autonomous: ' + str(self.is_autonomous))

    def callback_lidar_controller(self, msg):
        # self.lidar_message = msg
        if self.is_autonomous:
            if not msg.angular.z == 0:
                self.object_detected = True
                self.publish_to_pilot(msg)
            else:
                self.object_detected = False


if __name__ == '__main__':
    rospy.init_node('stern4most_pilot_AI2')
    rospy.loginfo('node stern4most_pilot_AI2 has been initialized')
    Pilot()
    rospy.spin()
