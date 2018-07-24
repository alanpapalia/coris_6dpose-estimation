import logging
logging.basicConfig(level=logging.INFO)

import os
import cv2
import numpy as np
import sys
import PIL
from PIL import Image
from numpy import *
from matplotlib import pyplot as plt
from skimage.measure import compare_ssim

depth_folder = './frames/depth/'
color_folder = './frames/color/'
gray_folder = './frames/gray/'
pts_folder = './frames/points/'
cleaned_image_folder = './frames/seg/'

# cv2.namedWindow('Test Window')

def getCheckerboardCorners(grayImg, pSize):
    ret, corners = cv2.findChessboardCorners(grayImg, pSize, cv2.CALIB_CB_ADAPTIVE_THRESH)
    # ret, corners = cv2.findChessboardCorners(grayImg, pSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    # ret, corners = cv2.findChessboardCorners(grayImg, pSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    print corners
    if (corners is not None):
        print corners.shape
        corners = corners[:,0,:]
        np.savetxt('corners.txt', corners)
    return ret, corners

def clearTestImages(nCams):
    if nCams == 1:
        depth_folder = './frames/single_camera/depth/'
        color_folder = './frames/single_camera/color/'
        gray_folder = './frames/single_camera/gray/'
        pts_folder = './frames/single_camera/points/'
        dirs = [depth_folder, color_folder, gray_folder, pts_folder]
    elif nCams == 2:
        depth_folder1 = './frames/two_camera/depth1/'
        color_folder1 = './frames/two_camera/color1/'
        gray_folder1 = './frames/two_camera/gray1/'
        pts_folder1 = './frames/two_camera/points1/'
        depth_folder2 = './frames/two_camera/depth2/'
        color_folder2 = './frames/two_camera/color2/'
        gray_folder2 = './frames/two_camera/gray2/'
        pts_folder2 = './frames/two_camera/points2/'
        dirs = [depth_folder1, color_folder1, gray_folder1, pts_folder1,
                depth_folder2, color_folder2, gray_folder2, pts_folder2]

    for folder in dirs:
        for file in os.listdir(folder):
            file_path = os.path.join(folder, file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(e)


def processDepthImage(img):
    print()


def saveDepthImageAsTXT(arr, name):
    # save depth img to text for analysis
    np.savetxt('./frames/depth/full_proc_'+name+'.txt', arr)


def checkDepthImage(img):
    nRow, nCol = img.shape
    f = open('test.txt', 'w')
    for r in xrange(nRow):
        for c in xrange(nCol):
            pVal = img[r][c]
            if pVal > 2:
                print(pVal)
            f.write('{:3}'.format(pVal))
        f.write('\n')


def colBasedDepImgSeg():
    print('Processing images...')
    for file_name in os.listdir(gray_folder):
        imageGray = cv2.imread(gray_folder + file_name, 0)
        imageDepth = cv2.imread(depth_folder + file_name, 2)

        for i in range(len(imageGray)):
            for j in range(len(imageGray[0])):
                if imageGray[i][j] < 20:
                    imageGray[i][j] = 255  # maybe don't want?
                    imageDepth[i][j] = 0

        cv2.imwrite(cleaned_image_folder + 'gray_' +
                    file_name, imageGray)  # maybe don't want?
        cv2.imwrite(cleaned_image_folder + 'depth_' + file_name, imageDepth)
        saveDepthImageAsTXT(imageDepth, file_name[:-4])


def clearImages():
    for file in os.listdir(cleaned_image_folder):
        file_path = os.path.join(cleaned_image_folder, file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)


"""
Segments color image based on watershed algorithm.
Is intended to be used after image has thresholded out all black
"""


def watershedSegment(colImg):
    img = colImg
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (5,5), 0)
    ret, thresh = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    # gray, 40, 255, cv2.THRESH_OTSU)
    # cv2.imshow('thresh test', thresh)
    # noise removal
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=3)
    cv2.imshow('sure bg', sure_bg)

    # Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(
        dist_transform, 0.7*dist_transform.max(), 255, 0)
    # cv2.imshow('sure fg', sure_fg)

    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    cv2.imshow('sure bg', sure_bg)

    # Marker labelling
    ret, markers = cv2.connectedComponents(sure_fg)

    # Add one to all labels so that sure background is not 0, but 1
    markers = markers+1

    # Now, mark the region of unknown with zero
    markers[unknown == 255] = 0

    markers = cv2.watershed(img, markers)
    img[markers == -1] = [255, 0, 0]
    return img


"""Receives BGR color image and thresholds to remove all black objects and turn to white

[description]
"""


def threshColImg(colImg):
    grayscale = cv2.cvtColor(colImg, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('pre thresh', grayscale)
    ret, g2 = cv2.threshold(grayscale, 35, 255, cv2.THRESH_TOZERO)
    # cv2.imshow('gray_adapt_thresh', g2)

    row, col, dep = colImg.shape
    for r in xrange(row):
        for c in xrange(col):
            if g2[r][c] == 0:
                colImg[r][c] = (0, 0, 0)
    return


""" Returns list of frames from video in chronological
order """


def getFrameList(frameDir):
    frames = []

    for f in os.listdir(frameDir):
        try:
            frame = cv2.imread(frameDir+f)
            # cv2.imshow('getList', frame)
            frames.append(frame)
        except Exception as e:
            print(e)

    return frames


""" Runs background subtraction on images """


def runBGSubMOG2(frames, transition_threshold):

    fgbg = cv2.createBackgroundSubtractorMOG2()
    prev = frames[0]
    fgmask = fgbg.apply(prev)
    cv2.namedWindow('original')
    cv2.namedWindow('bg subtracted')
    transitions = []

    for (i, next) in enumerate(frames[1:]):
        prev_gray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
        next_gray = cv2.cvtColor(next, cv2.COLOR_BGR2GRAY)
        similarity_metric = compare_ssim(prev_gray, next_gray)
        # print('prev/next similarity measure = %f' % similarity_metric)
        # if similarity_metric < transition_threshold:
        fgmask = fgbg.apply(next)
        fgdn = denoise_foreground(next, fgmask)
        transitions.append((1, fgdn))
        # else:
        #     fgmask = fgbg.apply(next)
        #     transitions.append((0, None))
        prev = next.copy()
    return transitions


"""Method pulled from online

[description]
"""


def denoise_foreground(img, fgmask):
    img_bw = 255*(fgmask > 5).astype('uint8')
    se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    mask = cv2.morphologyEx(img_bw, cv2.MORPH_CLOSE, se1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)
    mask = np.dstack([mask, mask, mask]) / 255
    img_dn = img * mask
    return img_dn


""" Make a new directory given filepath

[description]
"""


def makeFolder(direct):
    try:
        if not os.path.exists(direct):
            os.makedirs(direct)
    except OSError:
        print("Cannot make directory: " + direct)


""" Takes a given directory containing RGB frames
and writes grayscale versions to directory sharing same parent
directory
"""


def makeGrayFrames(frameDir):
    grayDir = frameDir + "../grayFrames/"
    makeFolder(grayDir)
    for f in os.listdir(frameDir):
        try:
            colFrame = cv2.imread(frameDir+f)
            grayFrame = cv2.cvtColor(colFrame, cv2.COLOR_RGB2GRAY)
            cv2.imwrite(grayDir+f, grayFrame)
        except Exception as e:
            print(e)


"""Takes grayscale image and converts it
such that mostly black and mostly white
pixels are made to all black based on
simple thresholding of highest and lowest vals
"""


def simpleThreshMaskBlkAndWhite(grayImg):
    img = grayImg.copy()
    ret, img = cv2.threshold(img, 40, 255, cv2.THRESH_TOZERO)
    ret, img = cv2.threshold(img, 200, 255, cv2.THRESH_TOZERO_INV)
    return img


"""Takes grayscale images and masks all black
and white squares from checkerboard based on
size (num pixels) and distance. Works assuming
that squares are aligned with image orientation
"""


def maskSmallWhtSquares(grayImg):
    row, col = grayImg.shape
    markers = np.zeros((row, col))

    cnt = 1

    # vars to track size of square to mark sizes
    sqrWid = 0
    sqrHt = 0
    r = 0
    c = 0
    # iterate through all pixels in image
    while r < row:
        while c < col:
            sqrWid = 0
            sqrHt = 0
            if markers[r][c] == 0 and grayImg[r][c] > 10:
                # iterate through pixels to the right and down
                # until markedly different grayvals
                while abs(grayImg[r][c]-grayImg[r+sqrWid][c]) < 50:
                    if r+sqrWid == 479:
                        break
                    else:
                        sqrWid += 1
                while abs(grayImg[r][c]-grayImg[r][c+sqrHt]) < 50:
                    if c+sqrHt == 639:
                        break
                    else:
                        sqrHt += 1
                # if small enough area and approx square, black
                # out the object
                print("size: " + str(sqrWid*sqrHt))
                if sqrWid*sqrHt > 15 and sqrWid*sqrHt < 250 and abs(sqrHt-sqrWid < 100):
                    for x in xrange(sqrWid):
                        for y in xrange(sqrHt):
                            markers[r+x][c+y] = 1
                            grayImg[r+x][c+y] = 0
                print(cnt)
                cnt += 1
            r += sqrWid
            c += 1
        r += 1
        c = 0
    return grayImg


def watershedSeg(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    # noise removal
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    # Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(
        dist_transform, 0.7*dist_transform.max(), 255, 0)

    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # Marker labelling
    ret, markers = cv2.connectedComponents(sure_fg)

    # Add one to all labels so that sure background is not 0, but 1
    markers = markers+1

    # Now, mark the region of unknown with zero
    markers[unknown == 255] = 0

    markers = cv2.watershed(img, markers)
    img[markers == -1] = [255, 0, 0]
