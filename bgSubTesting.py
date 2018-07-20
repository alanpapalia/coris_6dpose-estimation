import image_controls as imControl
import cv2
import os
from watershed import Watershed
from PIL import Image
import matplotlib.pyplot as plt


""" Takes directory of gray frames and masks
them such that mostly black and mostly white
pixels are made to all black
"""
def maskAllGrayInDir(grayFrameDir):
    for f in os.listdir(grayFrameDir):
    	print("Working on: " + f)
        img = cv2.imread(grayFrameDir+f,0)
        img = imControl.simpleThreshMaskBlkAndWhite(img)
        # img = imControl.maskSmallWhtSquares(img)
        img = watershedGrayImg(img)
        cv2.imwrite(grayFrameDir+f, img)


""" Attempt a watershed segmentation
of color images
"""

def watershedGrayImg(gImg):
	w = Watershed()
	labels = w.apply(gImg)
	return labels

baseDir = './frames/single_camera/color/'
tDir = raw_input("Which trial would you like to test? (t1/t2/t3/t4)\n")
frameDir = baseDir+tDir+'/frameData/'
grayFrameDir = baseDir+tDir+'/grayFrames/'

imControl.makeGrayFrames(frameDir)

maskAllGrayInDir(grayFrameDir)

# frames = imControl.getFrameList(frameDir)
# bgSubs = imControl.runBGSubMOG2(frames, 0.5)

# cv2.namedWindow('original')
# cv2.namedWindow('bg subtracted')
# for x in xrange(len(frames)):
#     cv2.imshow('original',frames[x])
#     cv2.imshow('bg subtracted', bgSubs[x])






