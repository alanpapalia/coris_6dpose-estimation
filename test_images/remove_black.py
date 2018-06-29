import numpy as np
import cv2

tracker = 570


for tracker in range(570, 960, 30):
    imageRGB = cv2.imread('/home/eadom/test_images/frames/colorFrame' + str(tracker) + '.jpg', 1)
    imageDepth = cv2.imread('/home/eadom/test_images/frames/depthFrame' + str(tracker) + '.jpg', 2)

    for i in range(len(imageRGB)):
        for j in range(len(imageRGB[0])):
            if imageRGB[i][j][0] < 50 and imageRGB[i][j][1] < 50 and imageRGB[i][j][2] < 50:
                imageRGB[i][j] = [0, 0, 0]
                imageDepth[i][j] = 0

    cv2.imwrite('/home/eadom/test_images/frames/colorFrameNOHAND' + str(tracker) + '.jpg', imageRGB)
    cv2.imwrite('/home/eadom/test_images/frames/depthFrameNOHAND' + str(tracker) + '.jpg', imageDepth)
