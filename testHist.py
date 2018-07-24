import cv2
import numpy as numpy
from matplotlib import pyplot as plt
import os
import time


for f in os.listdir('./'):
    img = cv2.imread(f, 0)
    plt.hist(img.ravel(),256,[0,256])
    plt.ylim(0,8000)
    plt.savefig('hist_'+f[:-3]+'.png')



    # plt.show()

            