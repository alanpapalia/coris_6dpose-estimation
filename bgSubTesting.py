import image_controls as imControl
import cv2
import os
from watershed import Watershed
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

# Code here is for the purpose of testing
# various background subtraction methods


""" Takes directory of gray frames and masks
them such that mostly black and mostly white
pixels are made to all black
"""


def maskAllGrayInDir(grayFrameDir):
    for f in os.listdir(grayFrameDir):
        print("Working on: " + f)
        img = cv2.imread(grayFrameDir+f, 0)
        img = imControl.simpleThreshMaskBlkAndWhite(img)
        # img = imControl.maskSmallWhtSquares(img)
        img = watershedGrayImg(img)
        cv2.imwrite(grayFrameDir+f, img)


def cannyDetectAllGrayInDir(grayFrameDir):
    for f in os.listdir(grayFrameDir):
        print("Working on: " +  grayFrameDir + f)
        img = cv2.imread(grayFrameDir+f, 0)
        # ret, img = cv2.threshold(img, 20, 255, cv2.THRESH_TOZERO)
        # ret, img = cv2.threshold(img, 220, 255, cv2.THRESH_TOZERO_INV)
        img = cv2.Canny(img, 70, 120)

        # img = imControl.maskSmallWhtSquares(img)
        cv2.imwrite(grayFrameDir+"canny_"+f, img)


def cannyDetectDepth(depthDir):
    for f in os.listdir(depthDir):
        print("Working on: " +  depthDir + f)
        img = cv2.imread(depthDir+f, 0)
        img = cv2.multiply(img, np.array([30]))
        # ret, img = cv2.threshold(img, 20, 255, cv2.THRESH_TOZERO)
        # ret, img = cv2.threshold(img, 220, 255, cv2.THRESH_TOZERO_INV)
        img = cv2.Canny(img, 0, 255)

        # img = imControl.maskSmallWhtSquares(img)
        cv2.imwrite(depthDir+"canny_"+f, img)

""" iterates through all grayscale
images created and calculates/draws
checkerboard corners for them. Also
writes the corners to .txt file
"""

def findCornersAllGrayInDir(grayFrameDir):
    pSize = (9,8)
    for f in os.listdir(grayFrameDir):
        print("Working on: " + f)
        img = cv2.imread(grayFrameDir+f, 0)
        ret, corners = cv2.findChessboardCorners(img, pSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
        cv2.drawChessboardCorners(img, pSize, corners, ret  )
        corners = corners[:,0,:]
        np.savetxt('corners.txt', corners)

        # img = imControl.maskSmallWhtSquares(img)
        cv2.imwrite(grayFrameDir+f, img)

""" Attempt a watershed segmentation
of color images based on code pulled from github

--results were poor, overly segmented
"""
def watershedGrayImg(gImg):
    w = Watershed()
    labels = w.apply(gImg)
    return labels


# baseDir = './frames/single_camera/'
# tDir = raw_input("Which trial would you like to test? (t1/t2/t3/t4)\n")
# frameDir = baseDir+tDir+'/color/'
# grayFrameDir = baseDir+tDir+'/gray/'

# imControl.makeGrayFrames(frameDir)

# cannyDetectAllGrayInDir(grayFrameDir)
# findCornersAllGrayInDir(grayFrameDir)
# frames = imControl.getFrameList(frameDir)
# bgSubs = imControl.runBGSubMOG2(frames, 0.5)

# cv2.namedWindow('original')
# cv2.namedWindow('bg subtracted')
# for x in xrange(len(frames)):
#     cv2.imshow('original',frames[x])
#     cv2.imshow('bg subtracted', bgSubs[x])
