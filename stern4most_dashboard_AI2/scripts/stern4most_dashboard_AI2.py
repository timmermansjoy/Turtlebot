#!/usr/bin/env python3

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel
from PyQt5.QtCore import QTimer


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import cv2
from cv_bridge import CvBridge, CvBridgeError

from threading import Lock

import numpy as np

import sys
import os


GUI_UPDATE_PERIOD = 16  # ms
BACKWARDS = True


class stern4most_dashboard_AI2(QWidget):

    def __init__(self):
        super(stern4most_dashboard_AI2, self).__init__()

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.init_subscriber()

        # ---- Subscribers ----
        self.ranking_sub = rospy.Subscriber('dashboard_ranking', String, self.callback_ranking)
        rospy.loginfo('subscribed to topic dashboard_ranking')

        # ---- Publishers ----
        self.controller_pub = rospy.Publisher('manual_controller', Twist, queue_size=10)
        rospy.loginfo('created publisher for topic manual_controller')
        self.vel = Twist()

        self.manual_autonomous_pub = rospy.Publisher('manual_autonomous', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic manual_autonomous')
        self.is_autonomous = Bool()

        self.sternformost_pub = rospy.Publisher('sternformost', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic sternformost')
        self.sternformost = Bool()

        self.recording_pub = rospy.Publisher('record', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic record')
        self.recording = Bool()

        self.visionmode_pub = rospy.Publisher('visionmode', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic visionmode')
        self.visionMode = Bool()

        self.AI_pub = rospy.Publisher('drive_ai', Bool, queue_size=10)
        rospy.loginfo('created publisher for topic drive_ai')
        self.AI = Bool()

        # Setup the GUI and start its threading
        self.init_gui()

    # ---- INIT methods ----

    def init_subscriber(self):
        self.ros_image_lock = Lock()

        self.ros_image_lock.acquire()
        try:
            self.received_new_data = False
        finally:
            self.ros_image_lock.release()

        # Start to listen...
        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')

    def init_gui(self):
        # Add layouts for the image, buttons and ranking
        self.page_layout = QtWidgets.QVBoxLayout()
        self.image_layout = QtWidgets.QHBoxLayout()
        self.ranking_layout = QtWidgets.QVBoxLayout()
        self.button_layout = QtWidgets.QGridLayout()

        self.page_layout.addLayout(self.image_layout)
        self.page_layout.addLayout(self.button_layout)
        self.image_layout.addLayout(self.ranking_layout)

        self.image_frame = QtWidgets.QLabel()
        self.image_layout.addWidget(self.image_frame)

        self.create_labels()

        self.create_buttons()

        self.setLayout(self.page_layout)

        # Start to update the image on the gui.
        self.gui_timer = QTimer(self)
        self.gui_timer.start(GUI_UPDATE_PERIOD)
        self.gui_timer.timeout.connect(self.update_image_on_gui)

    # ---- Callbacks ----

    def callback_image_raw(self, image):
        self.ros_image_lock.acquire()
        try:
            self.ros_image = image
            self.received_new_data = True
        finally:
            self.ros_image_lock.release()

    def callback_ranking(self, msg):
        r, s, total_time, last_sector_time = self.parse_ranking(msg.data)

        self.round.setText('Round: ' + r)
        self.round.resize(self.round.sizeHint())

        self.sector.setText('Sector: ' + s)
        self.sector.resize(self.sector.sizeHint())

        self.total_time.setText('Total time: ' + total_time)
        self.total_time.resize(self.total_time.sizeHint())

        self.last_sector_time.setText('Last sector time: ' + last_sector_time)
        self.last_sector_time.resize(self.last_sector_time.sizeHint())

        self.rate.sleep()

    # ---- Helpers ----

    def create_buttons(self):
        self.forward_button = QPushButton("Forward")
        self.forward_button.clicked.connect(self.forward_button_clicked)
        self.button_layout.addWidget(self.forward_button, 0, 1)

        self.backward_button = QPushButton("Backward")
        self.backward_button.clicked.connect(self.backward_button_clicked)
        self.button_layout.addWidget(self.backward_button, 2, 1)

        self.left_button = QPushButton("Left")
        self.left_button.clicked.connect(self.left_button_clicked)
        self.button_layout.addWidget(self.left_button, 1, 0)

        self.right_button = QPushButton("Right")
        self.right_button.clicked.connect(self.right_button_clicked)
        self.button_layout.addWidget(self.right_button, 1, 2)

        self.hard_left_button = QPushButton("hard-left")
        self.hard_left_button.clicked.connect(self.hard_left_button_clicked)
        self.button_layout.addWidget(self.hard_left_button, 0, 0)

        self.hard_right_button = QPushButton("hard-right")
        self.hard_right_button.clicked.connect(self.hard_right_button_clicked)
        self.button_layout.addWidget(self.hard_right_button, 0, 2)

        self.stop_button = QPushButton("STOP")
        self.stop_button.clicked.connect(self.stop_button_clicked)
        self.button_layout.addWidget(self.stop_button, 1, 1)

        self.autonomous_button = QPushButton("GO TURTLE GOOO")
        self.autonomous_button.clicked.connect(self.autonomous_button_clicked)
        self.button_layout.addWidget(self.autonomous_button, 2, 0)

        self.sternformost_button = QPushButton("Drive sternformost")
        self.sternformost_button.clicked.connect(self.sternformost_button_clicked)
        self.button_layout.addWidget(self.sternformost_button, 2, 2)

        self.record_button = QPushButton("Record")
        self.record_button.clicked.connect(self.recording_button_clicked)
        self.button_layout.addWidget(self.record_button, 4, 2)

        self.vision_button = QPushButton("Vision")
        self.vision_button.clicked.connect(self.vision_button_clicked)
        self.button_layout.addWidget(self.vision_button, 4, 0)

        self.AI_button = QPushButton("AI")
        self.AI_button.clicked.connect(self.AI_button_clicked)
        self.button_layout.addWidget(self.AI_button, 4, 1)

    def create_labels(self):
        self.player_name = QtWidgets.QLabel()
        self.player_name.setText('Name: AI2')
        self.ranking_layout.addWidget(self.player_name)

        self.round = QtWidgets.QLabel()
        self.round.setText('Round: 1')
        self.ranking_layout.addWidget(self.round)

        self.sector = QtWidgets.QLabel()
        self.sector.setText('Sector: 1')
        self.ranking_layout.addWidget(self.sector)

        self.total_time = QtWidgets.QLabel()
        self.total_time.setText('Total time: 00:00')
        self.ranking_layout.addWidget(self.total_time)

        self.last_sector_time = QtWidgets.QLabel()
        self.last_sector_time.setText('Last sector time: 00:00')
        self.ranking_layout.addWidget(self.last_sector_time)

    def parse_ranking(self, data):
        message = data.strip('[]()')
        rospy.loginfo(message)

        message = message.replace('\'', '')
        rospy.loginfo(message)

        ranking = message.replace(' ', '').split(',')
        rospy.loginfo(ranking)

        r = ranking[1]
        s = ranking[2]
        total_time = round(float(ranking[3]), 2)
        min = int(total_time // 60)
        sec = int(total_time % 60)
        min = f'0{min}' if min < 10 else min
        sec = f'0{sec}' if sec < 10 else sec
        total_time = f'{min}:{sec}'

        last_sector_time = round(float(ranking[4]), 2)
        min = int(last_sector_time // 60)
        sec = int(last_sector_time % 60)
        min = f'0{min}' if min < 10 else min
        sec = f'0{sec}' if sec < 10 else sec
        last_sector_time = f'{min}:{sec}'

        return r, s, str(total_time), str(last_sector_time)

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
        qt_image = QtGui.QImage(
            rgb_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(qt_image))

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    # ---- Button Handlers ----

    def stop_button_clicked(self):
        self.is_autonomous.data = False
        rospy.loginfo('advertising to topic is_autonomous with value ' + str(self.is_autonomous.data))
        self.manual_autonomous_pub.publish(self.is_autonomous)

        self.sternformost.data = False
        rospy.loginfo('advertising to topic sternformost with value ' + str(self.sternformost.data))
        self.sternformost_pub.publish(self.sternformost)

        self.AI.data = False
        rospy.loginfo('advertising to topic drive_ai with value ' + str(self.AI.data))
        self.AI_pub.publish(self.AI)

        self.vel.linear.x = 0
        self.vel.angular.z = 0

        rospy.loginfo('Stopping Turtlebot...')
        self.controller_pub.publish(self.vel)

    def forward_button_clicked(self):
        self.move_waffle(0.2, 0)

    def backward_button_clicked(self):
        self.move_waffle(-0.2, 0)

    def left_button_clicked(self):
        self.move_waffle(0, 0.02)

    def right_button_clicked(self):
        self.move_waffle(0, -0.02)

    def hard_left_button_clicked(self):
        self.move_waffle(0, 0.2)

    def hard_right_button_clicked(self):
        self.move_waffle(0, -0.2)

    def move_waffle(self, line_vel, ang_vel):
        self.is_autonomous.data = False
        rospy.loginfo('advertising to topic manual_autonomous with value ' + str(self.is_autonomous.data))
        self.manual_autonomous_pub.publish(self.is_autonomous)

        self.sternformost.data = False
        rospy.loginfo('advertising to topic sternformost with value ' + str(self.sternformost.data))
        self.sternformost_pub.publish(self.sternformost)

        self.AI.data = False
        rospy.loginfo('advertising to topic drive_ai with value ' + str(self.AI.data))
        self.AI_pub.publish(self.AI)

        if self.vel.linear.x + line_vel <= 0.22:
            self.vel.linear.x += line_vel
        else:
            self.vel.linear.x = 0.22
        if self.vel.linear.x < -0.2:
            self.vel.linear.x = -0.2
        if self.vel.angular.z + ang_vel < 0.22:
            self.vel.angular.z += ang_vel
        else:
            self.vel.angular.z = 0.22

        rospy.loginfo('advertising to topic controller with linear x value ' + str(self.vel.linear.x) + ' and angular z value of ' + str(self.vel.angular.z))
        self.controller_pub.publish(self.vel)

        self.rate.sleep()

    def autonomous_button_clicked(self):
        self.is_autonomous.data = True
        rospy.loginfo('advertising to topic manual_autonomous with value ' + str(self.is_autonomous.data))
        self.manual_autonomous_pub.publish(self.is_autonomous)

        self.sternformost.data = False
        rospy.loginfo('advertising to topic sternformost with value ' + str(self.sternformost.data))
        self.sternformost_pub.publish(self.sternformost)

        self.AI.data = False
        rospy.loginfo('advertising to topic drive_ai with value ' + str(self.AI.data))
        self.AI_pub.publish(self.AI)

        self.rate.sleep()

    def sternformost_button_clicked(self):
        self.is_autonomous.data = True
        rospy.loginfo('advertising to topic manual_autonomous with value ' + str(self.is_autonomous.data))
        self.manual_autonomous_pub.publish(self.is_autonomous)

        self.sternformost.data = True
        rospy.loginfo('advertising to topic sternformost with value ' + str(self.sternformost.data))
        self.sternformost_pub.publish(self.sternformost)

        self.AI.data = False
        rospy.loginfo('advertising to topic drive_ai with value ' + str(self.AI.data))
        self.AI_pub.publish(self.AI)

        self.rate.sleep()

    def AI_button_clicked(self):
        self.is_autonomous.data = True
        rospy.loginfo('advertising to topic manual_autonomous with value ' + str(self.is_autonomous.data))
        self.manual_autonomous_pub.publish(self.is_autonomous)

        self.sternformost.data = False
        rospy.loginfo('advertising to topic sternformost with value ' + str(self.sternformost.data))
        self.sternformost_pub.publish(self.sternformost)

        self.AI.data = True
        rospy.loginfo('advertising to topic drive_ai with value ' + str(self.AI.data))
        self.AI_pub.publish(self.AI)

        self.rate.sleep()

    def recording_button_clicked(self):
        self.recording.data = True
        rospy.loginfo('advertising to topic record with value ' + str(self.recording.data))
        self.recording_pub.publish(self.recording)

        self.rate.sleep()

    def vision_button_clicked(self):
        self.visionMode.data = True
        rospy.loginfo('advertising to topic visionmode with value ' + str(self.visionMode.data))
        self.visionmode_pub.publish(self.visionMode)

        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("stern4most_dashboard_AI2")
    rospy.loginfo('Node stern4most_dashboard_AI2 has been initialized')

    application = QApplication(sys.argv)
    gui = stern4most_dashboard_AI2()
    gui.show()
    sys.exit(application.exec_())
