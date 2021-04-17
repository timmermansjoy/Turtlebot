import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

import data_collection
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

maxThrottle = 0.3
record = 0

GUI_UPDATE_PERIOD = 0.10  # Seconds


class AI:

    def __init__(self):
        self.running = True

        # ---- Subscribers ----
        self.video_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image_raw)
        rospy.loginfo('subscribed to topic /camera/rgb/image_raw')

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.loginfo("Subscribed to topic /scan")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = None
        self.imageLock = Lock()
        self.lidar_message = Twist()
        self.BACKWARDS = Bool()
        self.statusMessage = ''
        self.connected = False
        self.redrawTimer = rospy.Timer(rospy.Duration(GUI_UPDATE_PERIOD), self.callback_redraw)

    def callback_image_raw(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    def convert_ros_to_opencv(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as error:
            raise Exception("Failed to convert to OpenCV image")

    def main(self):
        # Preprocess the image
        if self.running == True and self.image is not None:
            self.imageLock.acquire()
            try:
                # Convert the captured frame from ROS to OpenCV.
                image_cv = self.convert_ros_to_opencv(self.image)
            finally:
                self.imageLock.release()
            image_cv = cv2.resize(image_cv, dsize=(228, 156), interpolation=cv2.INTER_CUBIC)

        # Get stearing angle
        # ---- need to impliment ----
        steering = 0.2
        record == 0
        startRecording = False
        while True:
            if startRecording:  # later change to (if recording button is pressed)
                if record == 0:
                    print('Recording Started ...')
                record += 1
                sleep(0.300)
                if record == 1:
                    data_collection.saveData(image_cv, self.lidar_message, steering)
                elif record == 2:
                    data_collection.saveLog()
                    record = 0

            motor.move(throttle, -steering)


if __name__ == "__main__":
    rospy.init_node("AI_listener")
    AI()
    rospy.spin()
