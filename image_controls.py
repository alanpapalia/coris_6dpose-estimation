import logging
logging.basicConfig(level=logging.INFO)

import os
import cv2
import numpy as np
import sys
import PIL
from PIL import Image
from numpy import *

def clearTestImages():

    depth_folder = './frames/depth/'
    color_folder = './frames/color/'
    gray_folder = './frames/gray/'

    dirs = [depth_folder, color_folder, gray_folder]
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

def saveDepthImageAsTXT():
    print()

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
    for file_name in os.listdir(color_folder):
        imageRGB = cv2.imread(color_folder + file_name, 1)
        imageDepth = cv2.imread(depth_folder + file_name, 2)

        for i in range(len(imageRGB)):
            for j in range(len(imageRGB[0])):
                if imageRGB[i][j][0] < 50 and imageRGB[i][j][1] < 50 and imageRGB[i][j][2] < 50:
                    imageRGB[i][j] = [0, 0, 0]
                    imageDepth[i][j] = 0

        cv2.imwrite(cleaned_image_folder + 'color_' + file_name, imageRGB)
        cv2.imwrite(cleaned_image_folder + 'depth_' + file_name, imageDepth)

# img = 'frame20.jpg'
# dir = './frames/depth/'
# file = asarray(Image.open(dir + img))
# # print(file)
# checkDepthImage(file)

