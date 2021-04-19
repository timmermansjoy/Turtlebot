import os
import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
from sklearn.utils import shuffle
import cv2

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Convolution2D, Flatten, Dense 
from tensorflow.keras.optimizers import Adam 

import matplotlib.image as mpimg 
from imgaug import augmenters as iaaa

import random 

# STEP 1 - Initialisation of the data 
def getName(filePath):
    imgPathL = filePath.split('/')[-2:]
    imgPath = os.path.join(imgPathL[0], imgPathL[1])
    return imgPath    

def importDataInfo(path):
    columns = ['Center', 'Steering']
    numberOfFolders = len(os.listdir(path))//2
    data = pd.DataFrame()
    for x in range(4):
        newData = pd.read_csv(os.path.join(path, f'log_{x}.csv'), names = columns)
        newData['Center'] = newData['Center'].apply(getName)
        data = data.append(newData, True)
    print(' ')
    print('Total images imported: ' + str(data.shape[0]))
    return data


# STEP 2 - VISUALIZE AND BALANCE DATA
def balanceData(data,display=True):
    nBin = 31
    samplesPerBin =  300
    hist, bins = np.histogram(data['Steering'], nBin)
    if display:
        center = (bins[:-1] + bins[1:]) * 0.5
        plt.bar(center, hist, width=0.03)
        plt.plot((np.min(data['Steering']), np.max(data['Steering'])), (samplesPerBin, samplesPerBin))
        plt.title('Data Visualisation')
        plt.xlabel('Steering Angle')
        plt.ylabel('No of Samples')
        plt.show()
    removeindexList = []
    for j in range(nBin):
        binDataList = []
        for i in range(len(data["Steering"])):
            if data['Steering'][i] >= bins[j] and data['Steering'][i] <= bins[j + 1]:
                binDataList.append(i)
        binDataList = shuffle(binDataList)
        binDataList = binDataList[samplesPerBin:]
        removeindexList.extend(binDataList)

    print('Removed Images:', len(removeindexList))
    data.drop(data.index[removeindexList], inplace=True)
    print('Remaining Images:', len(data))
    if display:
        hist, _ = np.histogram(data['Steering'], (nBin))
        plt.bar(center, hist, width=0.03)
        plt.plot((np.min(data['Steering']), np.max(data['Steering'])), (samplesPerBin, samplesPerBin))
        plt.title('Balanced Data')
        plt.xlabel('Steering Angle')
        plt.ylabel('No of Samples')
        plt.show()
    return data
    
#### STEP 3 - PREPARE FOR PROCESSING
def loadData(path, data):
    print(data)
    imagesPath = []
    steering = []
    for i in range(len(data)):
        indexed_data = data.iloc[i]
        imagesPath.append( os.path.join(path,indexed_data[0]))
        steering.append(float(indexed_data[1]))
    imagesPath = np.asarray(imagesPath)
    steering = np.asarray(steering)
    return imagesPath, steering
