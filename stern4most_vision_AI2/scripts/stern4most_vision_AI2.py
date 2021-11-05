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

        # ---- Initial variables ----
        self.image = None
        self.gotYellow = False

        self.imageLock = Lock()
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.callback_redraw)

        # ---- Subscribers ----
        self.video_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')

        self.sternformost_sub = rospy.Subscriber('sternformost', Bool, self.callback_sternformost)
        rospy.loginfo('subscribed to topic sternformost')
        self.BACKWARDS = False

        self.visionmode_sub = rospy.Subscriber('visionmode', Bool, self.callback_visionmode)
        rospy.loginfo('subscribed to topic visionmode')
        self.visionMode = 0

        # ---- Publishers ----
        self.controller_pub = rospy.Publisher('autonomous_controller', Twist, queue_size=10)
        rospy.loginfo('created publisher for topic autonomous_controller')
        self.vel = Twist()

        self.sector_crossed_pub = rospy.Publisher('sector_crossed', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic sector_crossed')
        self.sector_crossed = Bool()
        self.sector_crossed.data = True

    # ---- Callbacks ----

    def callback_redraw(self, event):
        """
        looks at the image and:
        - Calculates the turning angle and publishes that to the pilot
        - Checks if we have driven over a checkpoint and publishes that to the referee
        returns: Nothing
        """

        # Preprocess the image
        if self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()
            image_cv = cv2.resize(image_cv, dsize=(800, 550), interpolation=cv2.INTER_CUBIC)

            # ---- Corner radius ----
            ang_val = utils.getLaneCurve(image_cv, self.BACKWARDS, self.visionMode)
            self.publish(ang_val)

            # to make sure the cv2 showimage is shown in utils getLaneCurve
            key = cv2.waitKey(5)
            if key == 27:  # Esc key top stop
                cv2.destroyAllWindows()

            # ---- Checkpoint ----
            if utils.checkPoint(image_cv) and not self.gotYellow:
                self.gotYellow = True
            elif not utils.checkPoint(image_cv) and self.gotYellow:
                self.gotYellow = False
                rospy.loginfo('SECTOR CROSSED')
                self.sector_crossed_pub.publish(self.sector_crossed)

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    def callback_sternformost(self, msg):
        self.BACKWARDS = msg.data

    def callback_visionmode(self, msg):
        if msg.data:
            self.visionMode = (self.visionMode + 1) % 3

    # ---- Helpers ----

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def publish(self, ang_val):
        self.vel.angular.z = ang_val
        self.vel.linear.x = 0.27
        if self.BACKWARDS:
            self.vel.linear.x = -0.1
        rospy.loginfo('publishing to topic autonomous_controller with x value: ' + str(self.vel.linear.x) + ' and z value: ' + str(self.vel.angular.z))
        self.controller_pub.publish(self.vel)


if __name__ == '__main__':
    rospy.init_node('stern4most_vision_AI2')
    rospy.loginfo('node stern4most_vision_AI2 has been initialized')
    Stern4most_vision_AI2()
    rospy.spin()
