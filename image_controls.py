import logging
logging.basicConfig(level=logging.INFO)

import os
import cv2
import numpy as np
import sys
import PIL
from PIL import Image
from numpy import *

depth_folder = './frames/depth/'
color_folder = './frames/color/'
gray_folder = './frames/gray/'
ir_folder = './frames/IR/'
pts_folder = './frames/points/'
cleaned_image_folder = './frames/seg/'

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
    np.savetxt('./frames/depth/full_proc_'+name+'.txt', arr)  # save depth img to text for analysis

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

        cv2.imwrite(cleaned_image_folder + 'gray_' + file_name, imageGray)  # maybe don't want?
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
