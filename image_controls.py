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

depth_folder = './frames/depth/'
color_folder = './frames/color/'
gray_folder = './frames/gray/'
ir_folder = './frames/IR/'
pts_folder = './frames/points/'
cleaned_image_folder = './frames/seg/'

# cv2.namedWindow('Test Window')



def clearTestImages():
    dirs = [depth_folder, color_folder, gray_folder, pts_folder, ir_folder]
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
