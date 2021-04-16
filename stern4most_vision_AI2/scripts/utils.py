#!/usr/bin/env python3

import cv2
import numpy as np

curvelist = []
avgVal = 2


def detected_tree_branch(img):
    wT, hT, c = img.shape
    # pointOfInterest = [220, 405, 35, 400] ==> oude waarden
    pointOfInterest = [215, 400, 30, 395]
    points = valTrackbars(pointOfInterest)
    imgTreeBranch = warpImg(img, points, wT, hT)

    lab = cv2.cvtColor(imgTreeBranch, cv2.COLOR_BGR2LAB)
    upper_val = np.array([110, 180, 135])
    lower_val = np.array([10, 115, 120])

    mask = cv2.inRange(lab, lower_val, upper_val)

    tree_branch_detected = np.sum(mask)

    if tree_branch_detected > 100:
        return True
    return False


def checkPoint(img):
    """
    if we ride over a checkpoint
    returns: boolean
    """

    # transform image (get bottom of image)
    wT, hT, c = img.shape
    pointOfIntrest = [214, 405, 0, 400]
    points = valTrackbars(pointOfIntrest)
    imgCheckpoint = warpImg(img, points, wT, hT)

    # detect color
    hsv = cv2.cvtColor(imgCheckpoint, cv2.COLOR_BGR2HSV)
    upper_val = np.array([41, 255, 255])
    lower_val = np.array([21, 100, 100])
    mask = cv2.inRange(hsv, lower_val, upper_val)

    # return if it has yellow in image
    hasYellow = np.sum(mask)
    if hasYellow > 0:
        return True
    return False


def thresholding(img):
    """
    Creates a binary image based on hsv values
    returns: converted image
    """

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    upper_val = np.array([0, 0, 100])
    lower_val = np.array([179, 60, 170])
    # fill image with mask
    mask = cv2.inRange(hsv, upper_val, lower_val)
    return mask


def warpImg(img, points, w, h, inv=False):
    """
    Warp the image so it has more of a top down perspective
    returns: converted image
    """

    pts1 = np.float32([points])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp


def valTrackbars(points, wT=770, hT=434):
    """
    puts the points of intrest into an array with 4 values to crop the image
    returns: array
    """
    widthTop = points[0]
    heightTop = points[1]
    widthBottom = points[2]
    heightBottom = points[3]
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                         (widthBottom, heightBottom), (wT-widthBottom, heightBottom)])
    return points


def getHistogram(img, minPer=0.1, display=False, region=1):
    """
    calculates the avarage spread of values within an image on the X-axis
    so that we can later get the curvature of this image
    returns: int
    """

    # split the image in a region
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:, :], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0]-int(intensity/255/region)), (255, 0, 255), 1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist
    return basePoint


def __stackImages(scale, imgArray):
    """
    puts array of images in a single window
    returns: array of images
    """
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


def __drawPoints(img, points):
    """put points of the intrests on the image to see where they are"""
    for x in range(0, 4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img


def getLaneCurve(img, BACKWARDS, display=0):
    """
    calculate the curvature road inside an image
    returns: int between -1 and 1
    optional: show all the used images to get the curvature
    """
    imgResult = img.copy()

    # step 1 get binary image
    imgThres = thresholding(img)

    # step 2 warp image to points of intrest
    hT, wT, c = img.shape
    # topWidth, topHeight, bottomWidth, bottomHeight
    pointOfIntrest = [270, 340, 0, 550] if BACKWARDS else [120, 370, 0, 550]
    points = valTrackbars(pointOfIntrest)
    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = __drawPoints(img, points)

    # step 3 get curvature of line in image
    midPoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    basePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.6)
    curveRaw = basePoint-midPoint

    # step 4 avarage curve rate
    curvelist.append(curveRaw)
    if len(curvelist) > avgVal:
        curvelist.pop(0)
    curve = int(sum(curvelist)/len(curvelist))
    
    # display menu
    if display != 0:
        imgInvWarp = warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT//3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = wT//2
        cv2.putText(imgResult, str(curve), (wT//2-80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT//2, midY), (wT//2+(curve*3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY-25), (wT // 2 + (curve * 3), midY+25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve//50), midY-10), (w * x + int(curve//50), midY+10), (0, 0, 255), 2)
    if display == 2:
        imgStacked = __stackImages(0.7, ([img, imgWarp, imgWarpPoints],
                                         [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)

    elif display == 1:
        cv2.imshow('Result', imgResult)

     # NORMALIZATION
    curve = curve/100
    if BACKWARDS:
        curve *= 4
    if curve > 1:
        curve = 1
    if curve < -1:
        curve = -1
    # return the curve value reversed so the robot drives
    return curve * -1
