#!/usr/bin/env python3

import cv2
import numpy as np

curvelist = []
avgVal = 10


def thresholding(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([0, 0, 100])
    upperWhite = np.array([179, 60, 170])
    maskedWhite = cv2.inRange(hsv, lowerWhite, upperWhite)
    return maskedWhite


def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32([points])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp


def valTrackbars(points, wT=770, hT=434):
    widthTop = points[0]
    heightTop = points[1]
    widthBottom = points[2]
    heightBottom = points[3]
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                         (widthBottom, heightBottom), (wT-widthBottom, heightBottom)])
    return points


def getHistogram(img, minPer=0.1, display=False, region=1):

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


def stackImages(scale, imgArray):
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


def getLaneCurve(img, display=2):

    imgResult = img.copy()

    # step 1 get binary image
    imgThres = thresholding(img)

    # step 2 warp image to points of intrest
    hT, wT, c = img.shape
    pointOfIntrest = [214, 223, 0, 393]
    points = valTrackbars(pointOfIntrest)
    imgWarp = warpImg(imgThres, points, wT, hT)

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
        imgStacked = stackImages(0.7, ([img, imgWarp, img],
                                       [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)

    elif display == 1:
        cv2.imshow('Resutlt', imgResult)

     # NORMALIZATION
    curve = curve/100
    if curve > 1:
        curve == 1
    if curve < -1:
        curve == -1
    # return the curve value
    return curve
