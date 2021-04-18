#!/usr/bin/env python3

import rospy
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

from threading import Lock
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import data_collection


maxThrottle = 0.3
record = 0
GUI_UPDATE_PERIOD = 0.10  # Seconds


class AI:
    def __init__(self):
        # Initial values
        self.running = True
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = None
        self.imageLock = Lock()
        self.lidar_message = Twist()
        self.STEERING = Twist()
        self.RECORD = False
        self.recording_step = 0
        self.statusMessage = ''
        self.connected = False
        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.collect_data_and_save)

        # ---- Subscribers ----
        self.video_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback_lidar_scan)
        rospy.loginfo("Subscribed to topic /scan")

        self.record_sub = rospy.Subscriber('record', Bool, self.callback_record)
        rospy.loginfo('subscribed to topic record')

        self.turning_sub = rospy.Subscriber('manual_controller', Twist, self.callback_steering)
        rospy.loginfo('subscribed to topic manual_controller')

    # ---- Callbacks ----
    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    def callback_lidar_scan(self, data):
        data_perSweep = []
        index = 0
        object_found = False
        for value in data.ranges:
            current_angle = data.angle_min + (data.angle_increment * index)
            values = [current_angle, value]
            data_perSweep.append(values)
            index += 1

    def callback_record(self, data):
        if data.data == True:
            self.RECORD = not self.RECORD

    def callback_steering(self, data):
        self.STEERING = data

    # ---- Helpers ----
    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def collect_data_and_save(self, event):
        # Preprocess the image
        if self.running == True and self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()
            image_cv = cv2.resize(image_cv, dsize=(228, 156), interpolation=cv2.INTER_CUBIC)

        if self.RECORD:  # later change to (if recording button is pressed)
            if self.recording_step == 0:
                rospy.loginfo('Recording ...')
                self.recording_step += 1
            sleep(0.300)
            if self.recording_step == 1:
                data_collection.saveData(image_cv, self.STEERING.angular.z)
        if not self.RECORD and self.recording_step == 1:
            data_collection.saveLog()
            self.recording_step = 0

        # motor.move(throttle, -steering)


if __name__ == "__main__":
    print("test")
    rospy.init_node("AI_listener")
    AI()
    rospy.spin()
