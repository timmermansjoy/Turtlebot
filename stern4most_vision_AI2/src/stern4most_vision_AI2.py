#!/usr/bin/env python3

import rospy
import time

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
        self.video_sub   = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        self.manual_autonomous_sub = rospy.Subscriber('manual_autonomous', Bool, self.callback_manual_autonomous)
        self.controller_pub = rospy.Publisher('controller', Twist, queue_size=10)
        self.vel = Twist()
        self.vel.linear.x = 0.10
        self.is_autonomous = False
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = None
        self.imageLock = Lock()

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
            image_cv = cv2.resize(image_cv, dsize=(770, 434), interpolation=cv2.INTER_CUBIC)
            self.vel.angular.z = utils.getLaneCurve(image_cv, 0)
            if self.is_autonomous:
                self.controller_pub.publish(self.vel)
                self.rate.sleep()

            key = cv2.waitKey(5)
            if key == 27: # Esc key top stop
                cv2.destroyAllWindows()
                self.running = False

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()
    
    def callback_manual_autonomous(self, msg):
        self.is_autonomous = not msg.data


if __name__=='__main__':
    rospy.init_node('stern4most_vision_AI2')

    display = Stern4most_vision_AI2()

    while display.is_running():
        time.sleep(5)
