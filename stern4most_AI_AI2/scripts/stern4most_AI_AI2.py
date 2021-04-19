#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import os
from tensorflow.keras.models import load_model
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

from threading import Lock
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image


class AI_driver:
    def __init__(self):
        self.steeringSen = 1.2
        self.image = None
        self.imageLock = Lock()
        self.bridge = CvBridge()
        self.vel = Twist()
        self.vel.linear.x = 0.22

        # Choose model and get its location
        self.model_name = 'first'
        self.model_path = self.find_model(self.model_name)

        # load model
        self.model = load_model(self.model_path)

        # ---- Subscribers ----
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')

        # ---- Publishers ----
        self.AI_controller_pub = rospy.Publisher('ai_controller', Twist, queue_size=10)
        rospy.loginfo('created publisher for topic autonomous_controller')

     # ---- Callbacks ----

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()
        self.main()

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def preProcess(self, img):
        img = img[54:120, :, :]
        img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        img = cv2.resize(img, (200, 66))
        img = img / 255
        return img

    def find_model(self, model_name):
        model_name = f'{model_name}.h5'
        path = os.path.realpath("")
        for root, dirs, files in os.walk(path):
            if model_name in files:
                return os.path.join(root, model_name)
        return None

    def main(self):
        if self.image is not None:
            img = self.convert_ros_to_opencv(self.image)
            img = cv2.resize(img, dsize=(228, 156), interpolation=cv2.INTER_CUBIC)
            img = np.asarray(img)
            img = self.preProcess(img)
            img = np.array([img])

            steering = float(self.model.predict(img))
            rospy.loginfo("{}".format(steering * self.steeringSen))

            self.vel.angular.z = steering * self.steeringSen
            self.AI_controller_pub.publish(self.vel)


if __name__ == "__main__":
    rospy.init_node("AI_driver")
    rospy.loginfo('AI_driver node has been initiated')
    AI_driver()
    rospy.spin()
