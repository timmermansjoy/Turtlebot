"""
- This module saves images and a log file.
- Images are saved in a folder.
- Folder should be created manually with the name "DataCollected"
- The name of the image and the steering angle is logged
in the log file.
- Call the saveData function to start.
- Call the saveLog function to end.
- If runs independent, will save ten images as a demo.
"""
import pandas as pd
import os
import cv2
import rospy
from datetime import datetime

global imgList, steeringList
countFolder = 0
count = 0
imgList = []
LIDARList = []
steeringList = []

# GET CURRENT DIRECTORY PATH
myDirectory = os.path.join(os.getcwd(), 'Data')

# CREATE A NEW FOLDER BASED ON THE PREVIOUS FOLDER COUNT
while os.path.exists(os.path.join(myDirectory, f'IMG{str(countFolder)}')):
    countFolder += 1
newPath = myDirectory + "/IMG"+str(countFolder)
os.makedirs(newPath)


def saveData(img, steering):
    """SAVE IMAGES IN THE FOLDER"""
    global imgList, steeringList
    now = datetime.now()
    timestamp = str(datetime.timestamp(now)).replace('.', '')
    fileName = os.path.join(newPath, f'Image_{timestamp}.jpg')
    cv2.imwrite(fileName, img)
    imgList.append(fileName)
    steeringList.append(round(steering, 2))


def saveLog():
    """SAVE LOG FILE WHEN THE SESSION ENDS"""
    # global imgList, LIDARList, steeringList
    rawData = {
        'Image': imgList,
        'Steering': steeringList
    }
    dataFrame = pd.DataFrame(rawData)
    dataFrame.to_csv(os.path.join(myDirectory, f'log_{str(countFolder)}.csv'), index=False, header=False)
    rospy.loginfo('Log Saved')
    rospy.loginfo('Total Images: ' + str(len(imgList)))
