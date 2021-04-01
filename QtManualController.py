#!/usr/bin/env python3

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel
from PyQt5.QtCore import QTimer


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

from threading import Lock

import numpy as np

import sys
import os


GUI_UPDATE_PERIOD = 100  # ms


class ROSOpenCVQt(QWidget):

    def __init__(self):
        super(ROSOpenCVQt, self).__init__()

        # To bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Setup the subscriber, thread safe
        self.init_subscriber()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel = Twist()

        # Setup the GUI and start its threading
        self.init_gui()

    def init_subscriber(self):
        self.ros_image_lock = Lock()

        self.ros_image_lock.acquire()
        try:
            self.received_new_data = False
        finally:
            self.ros_image_lock.release()

        # Start to listen...
        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_image_raw)

    def init_gui(self):
        # Add a place to show the image
        page_layout = QtWidgets.QVBoxLayout()
        image_layout = QtWidgets.QVBoxLayout()
        button_layout = QtWidgets.QGridLayout()

        page_layout.addLayout(image_layout)
        page_layout.addLayout(button_layout)

        self.image_frame = QtWidgets.QLabel()
        image_layout.addWidget(self.image_frame)

        self.forward_button = QPushButton("Forward")
        self.forward_button.clicked.connect(self.forward_button_clicked)

        self.backward_button = QPushButton("Backward")
        self.backward_button.clicked.connect(self.backward_button_clicked)

        self.left_button = QPushButton("Left")
        self.left_button.clicked.connect(self.left_button_clicked)

        self.right_button = QPushButton("Right")
        self.right_button.clicked.connect(self.right_button_clicked)

        self.stop_button = QPushButton("STOP")
        self.stop_button.clicked.connect(self.stop_button_clicked)

        button_layout.addWidget(self.forward_button, 0, 1)
        button_layout.addWidget(self.left_button, 1, 0)
        button_layout.addWidget(self.stop_button, 1, 1)
        button_layout.addWidget(self.right_button, 1, 2)
        button_layout.addWidget(self.backward_button, 2, 1)

        self.setLayout(page_layout)

        # Start to update the image on the gui.
        self.gui_timer = QTimer(self)
        self.gui_timer.start(GUI_UPDATE_PERIOD)
        self.gui_timer.timeout.connect(self.update_image_on_gui)

    def callback_image_raw(self, image):
        self.ros_image_lock.acquire()
        try:
            self.ros_image = image
            self.received_new_data = True
        finally:
            self.ros_image_lock.release()

    def update_image_on_gui(self):
        # Get a new image if there's one and make a copy of it.
        new_image = False
        self.ros_image_lock.acquire()
        try:
            if self.received_new_data == True:
                new_image = True
                opencv_image = self.convert_ros_to_opencv(self.ros_image)
                self.received_new_data = False
        finally:
            self.ros_image_lock.release()

        if not new_image:
            return

        scale = 0.4
        interpolation = cv2.INTER_AREA
        width = int(opencv_image.shape[1] * scale)
        height = int(opencv_image.shape[0] * scale)
        dimensions = (width, height)

        scaled_image = cv2.resize(opencv_image, dimensions, interpolation)

        # Conver the scaled image to a QImage and show it on the GUI.
        rgb_image = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_image.shape
        bytes_per_line = channels * width
        qt_image = QtGui.QImage(rgb_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(qt_image))

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def stop_button_clicked(self):
        self.move_waffle(self.vel.linear.x * -1, self.vel.angular.z * -1)

    def forward_button_clicked(self):
        self.move_waffle(0.05, 0)

    def backward_button_clicked(self):
        self.move_waffle(-0.05, 0)

    def left_button_clicked(self):
        self.move_waffle(0, 0.02)

    def right_button_clicked(self):
        self.move_waffle(0, -0.02)

    def move_waffle(self, line_vel, ang_vel):
        if self.vel.linear.x + line_vel <= 0.22:
            self.vel.linear.x += line_vel
        else:
            self.vel.linear.x = 0.22
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        if self.vel.angular.z + ang_vel < 0.22:
            self.vel.angular.z += ang_vel
        else:
            self.vel.angular.z = 0.22
        self.pub.publish(self.vel)
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sternformost_opencv_qt")

    application = QApplication(sys.argv)
    gui = ROSOpenCVQt()
    gui.show()
    sys.exit(application.exec_())
