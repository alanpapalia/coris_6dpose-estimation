import os
import numpy as np
import cv2
import time

"""Function written to track image based on the color frames written to the given directory

dir - directory containing color images
fps - frames per second, for giving more realistic feedback in showing the tracked video
"""
def trackColorImg(dir, fps):
    cnt = 0
    for file_name in os.listdir(dir):

        if cnt == 0:
            # take first frame of the video
            frame = cv2.imread(dir + file_name)

            # setup initial location of window
            r, h, c, w = 250, 90, 400, 125  # simply hardcoded the values
            track_window = (c, r, w, h)

            # set up the ROI for tracking
            roi = frame[r:r + h, c:c + w]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
            roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
            cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

            # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

            time.sleep(5)

        else:
            frame = cv2.imread(dir + file_name)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

            # apply meanshift to get the new location
            ret, track_window = cv2.CamShift(dst, track_window, term_crit)

            # Draw it on image
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            img2 = cv2.polylines(frame,[pts],True, 255,2)
            cv2.imshow('img2',img2)
            k = cv2.waitKey(60) & 0xff

            if cnt == 1:
                time.sleep(3)

            time.sleep(fps)
            cv2.imwrite(dir+'track/'+str(cnt)+".jpg",img2)
        cnt+= 1

"""
Takes directory to clear and number of frames to remove from beginning for sake of processing

dir - directory to remove frames from
nRem - number of frames to remove from start of directory
"""
def clearUntrackedImg(dir, nRem):
    cnt = 0
    for file_name in os.listdir(dir):
        if cnt >= nRem:
            break

        file_path = os.path.join(dir, file_name)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)

        cnt += 1

"""
Driver function to call tracking functions

dir - directory of color images
fps - frames per second of recorded video
nRem - number of frames to remove from start of recorded video
"""
def runColorTracking(dir, fps, nRem):
    clearUntrackedImg(dir, nRem)
    trackColorImg(dir, fps)