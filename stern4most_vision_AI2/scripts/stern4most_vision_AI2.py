#!/usr/bin/env python3

import rospy
import time
import logging

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from threading import Lock

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import utils

GUI_UPDATE_PERIOD = 0.10  # Seconds


class Stern4most_vision_AI2:

    def __init__(self):
        self.running = True
        self.video_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')
        self.topic_name = 'autonomous_controller'
        self.controller_pub = rospy.Publisher(self.topic_name, Twist, queue_size=10)
        rospy.loginfo('created publisher for topic autonomous_controller')
        self.sector_crossed_pub = rospy.Publisher('sector_crossed', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic sector_crossed')
        #self.lidar_controller_sub = rospy.Subscriber('lidar_controller', Twist, self.callback_lidar_controller)
        self.vel = Twist()
        self.gotYellow = False
        self.sector_crossed = Bool()
        self.sector_crossed.data = True
        self.vel.linear.x = 0.25
        self.bridge = CvBridge()
        self.rate = rospy.Rate(5)
        self.image = None
        self.imageLock = Lock()
        self.lidar_message = Twist()

        self.statusMessage = ''

        self.connected = False

        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.callback_redraw)

    def is_running(self):
        return self.running

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def callback_redraw(self, event):
        if self.running == True and self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()
            image_cv = cv2.resize(image_cv, dsize=(800, 550), interpolation=cv2.INTER_CUBIC)
            self.vel.angular.z = utils.getLaneCurve(image_cv, 1)
            if utils.checkPoint(image_cv) and not self.gotYellow:
                self.gotYellow = True
            elif not utils.checkPoint(image_cv) and self.gotYellow:
                self.gotYellow = False
                rospy.loginfo('SECTOR CROSSED')
                self.sector_crossed_pub.publish(self.sector_crossed)
            rospy.loginfo('advertising to topic autonomous_controller with linear x value of ' + str(self.vel.linear.x) + ' and angular z value of ' + str(self.vel.angular.z))
            self.controller_pub.publish(self.vel)

            # to make sure the cv2 showimage is shown in utils getLaneCurve
            key = cv2.waitKey(5)
            if key == 27:  # Esc key top stop
                cv2.destroyAllWindows()
                self.running = False

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    # def callback_lidar_controller(self, msg):
        #self.lidar_message = msg


if __name__ == '__main__':
    rospy.init_node('stern4most_vision_AI2')
    rospy.loginfo('node stern4most_vision_AI2 has been initialized')

    display = Stern4most_vision_AI2()

    while display.is_running():
        time.sleep(5)

    rospy.spin()
