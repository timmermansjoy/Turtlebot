#!/usr/bin/env python3


"""
  _____________________________
/ THIS FILE IS DEPRICATED       \
\ This is only in to show work  /
  -----------------------------
         \   ^__^
          \  (oo)\_______
             (__)\       )\/\
                 ||----w |
                 ||     ||

"""

import rospy
import time

from sensor_msgs.msg import Image
from threading import Lock

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


GUI_UPDATE_PERIOD = 0.10  # Seconds


class SternformostVision:

    def __init__(self):
        self.running = True
        self.subVideo = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)

        self.bridge = CvBridge()

        self.image = None
        self.imageLock = Lock()

        self.threshold1 = 127
        self.threshold2 = 255

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

    def canny_edge_detector(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def region_of_interest_mask(self, image):
        height = image.shape[0]
        width = image.shape[1]

        polygons = np.array([
            [(0, height - 100),
             (0, height - 200),
             (width / 2, height / 2),
             (width, height - 200),
             (width, height - 100)]
        ], dtype=np.int32)
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, polygons, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def display_lines(self, image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
        return line_image

    def make_coordinates(self, image, line_parameters):
        if isinstance(line_parameters, np.ndarray):
            slope, intercept = line_parameters
            y1 = image.shape[0] - 100
            y2 = int(y1 * (2/3))
            x1 = int((y1 - intercept)/slope)
            x2 = int((y2 - intercept)/slope)
            return np.array([x1, y1, x2, y2])
        else:
            return None

    def average_slope_interception(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            if y1 - y2 < -25 or y1 - y2 > 25:
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                intercept = parameters[1]
                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = self.make_coordinates(image, left_fit_average)
        right_line = self.make_coordinates(image, right_fit_average)
        if left_line is not None and right_line is not None:
            return np.array([left_line, right_line])
        else:
            return None

    def callback_redraw(self, event):
        if self.running == True and self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()

            cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
            img = cv2.resize(image_cv, (360, 480))
            cv2.imshow("Image", img)

            canny_edges = self.canny_edge_detector(img)
            canny_edges = cv2.resize(canny_edges, (360, 480))
            cropped = self.region_of_interest_mask(canny_edges)
            lines = cv2.HoughLinesP(cropped, 2, np.pi/180, 100, np.array([]), minLineLength=80, maxLineGap=25)
            if lines is not None:
                averaged_lines = self.average_slope_interception(img, lines)
            else:
                averaged_lines = None
            if averaged_lines is not None:
                line_image = self.display_lines(img, averaged_lines)
                image_with_lines = cv2.addWeighted(img, 0.8, line_image, 1, 1)
            else:
                image_with_lines = img
            cv2.imshow('line_image', image_with_lines)
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


if __name__ == '__main__':
    rospy.init_node('sternformost_example_vision')

    display = SternformostVision()

    while display.is_running():
        time.sleep(5)
