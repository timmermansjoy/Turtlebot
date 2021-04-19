#!/usr/bin/env python3

from std_msgs.msg import Bool, String
import sys
import logging
from geometry_msgs.msg import Twist
import rospy


class Pilot:
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

        self.drive_ai_sub = rospy.Subscriber('drive_ai', Bool, self.callback_drive_ai)
        rospy.loginfo('subscribed to topic drive_ai')

        self.ai_controller_sub = rospy.Subscriber('ai_controller', Twist, self.callback_ai_controller)
        rospy.loginfo('subscribed to topic ai_controller')

        self.rate = rospy.Rate(10)
        self.stop_vel = Twist()
        self.is_autonomous = False
        self.drive_ai = Bool()
        self.object_detected = False

    def callback_autonomous_controller(self, msg):
        if self.is_autonomous and not self.object_detected and not self.drive_ai.data:
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
        rospy.loginfo('new value is_autonomous: ' + str(self.is_autonomous))

    def callback_lidar_controller(self, msg):
        if self.is_autonomous:
            if not msg.angular.z == 0:
                self.object_detected = True
                self.publish_to_pilot(msg)
            else:
                self.object_detected = False

    def callback_drive_ai(self, msg):
        rospy.loginfo('changing drive_ai from ' + str(self.drive_ai.data) + ' to ' + str(msg.data))
        self.drive_ai = msg
        rospy.loginfo('new value' + str(self.drive_ai.data))

    def callback_ai_controller(self, msg):
        rospy.loginfo('received message from AI')
        if self.drive_ai.data:
            rospy.loginfo('publishing to pilot from ai_controller')
            self.publish_to_pilot(self.vel)


if __name__ == '__main__':
    rospy.init_node('stern4most_pilot_AI2')
    rospy.loginfo('node stern4most_pilot_AI2 has been initialized')
    Pilot()
    rospy.spin()
