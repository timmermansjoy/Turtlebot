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
        self.video_sub   = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        logging.info('subscribed to topic /camera/rgb/image_raw')
        self.manual_autonomous_sub = rospy.Subscriber('manual_autonomous', Bool, self.callback_manual_autonomous)
        logging.info('subscribed to topic manual_autonomous')
        self.controller_pub = rospy.Publisher('controller', Twist, queue_size=10)
        logging.info('created publisher for topic controller')
        self.vel = Twist()
        self.vel.linear.x = 0.88
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
            image_cv = cv2.resize(image_cv, dsize=(800,550), interpolation=cv2.INTER_CUBIC)
            self.vel.angular.z = utils.getLaneCurve(image_cv,0) * -1.2
            if self.is_autonomous:
                logging.info('advertising to topic controller with linear x value of ' + str(self.vel.linear.x) + ' and angular z value of ' + str(self.vel.angular.z))
                self.controller_pub.publish(self.vel)
                self.rate.sleep()

            # key = cv2.waitKey(5)
            # if key == 27: # Esc key top stop
            #     cv2.destroyAllWindows()
            #     self.running = False

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()
    
    def callback_manual_autonomous(self, msg):
        logging.info('received message on topic manual_autonomous with value ' + str(msg.data))
        self.is_autonomous = not msg.data
        if self.is_autonomous:
            logging.info('ready to start advertising to topic controller')
        else:
            logging.info('stopped advertising to topic controller')


if __name__=='__main__':
    logging.basicConfig(level=logging.INFO)
    rospy.init_node('stern4most_vision_AI2')
    logging.info('node stern4most_vision_AI2 has been initialized')

    display = Stern4most_vision_AI2()

    while display.is_running():
        time.sleep(5)

    rospy.spin()
