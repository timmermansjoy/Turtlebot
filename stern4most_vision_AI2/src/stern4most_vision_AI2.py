#!/usr/bin/env python3

import rospy
import time

from sensor_msgs.msg import Image
from threading import Lock

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import utils

GUI_UPDATE_PERIOD = 0.10  # Seconds


class Stern4most_vision_AI2:
    
    def __init__(self):
        self.running = True
        self.subVideo   = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)

        self.bridge = CvBridge()

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
            utils.getLaneCurve(image_cv, 1)

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


if __name__=='__main__':
    rospy.init_node('stern4most_vision_AI2')

    display = Stern4most_vision_AI2()

    while display.is_running():
        time.sleep(5)
