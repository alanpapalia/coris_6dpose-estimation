import logging
logging.basicConfig(level=logging.INFO)

import os
import shutil
import time
import numpy as np
import scipy
from scipy import misc
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option
from pyrealsense import offline
import image_controls as imControl
import pcl
import time
from datetime import datetime
import calendar
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from collections import deque
import signal
import multiprocessing



# Array should be in form of 3 x N
def transformPCD(arr, T):
    pcd = np.array([[0] * 3] * len(arr), dtype=np.float32)
    for i in range(0, len(arr)):
        pcd[i] = np.matmul(T, (list(arr[i]) + [1]))[0:3]
    return pcd

def savePCD(arr, fnameToWrite):
    p = pcl.PointCloud()
    p.from_array(np.array(arr,  dtype=np.float32))
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (10)
    fil.set_std_dev_mul_thresh (1.0)
    res = fil.filter()
    pcl.save(p, fnameToWrite)

def colWriter(colQueue, trialName):
    while True:
        item = colQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        cv2.imwrite("./frames/two_camera/trials/" + trialName + "/color%d/frame%d.jpg" % (camNum, frameNum), img)

def depWriter(depQueue, trialName):
    while True:
        item = depQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        np.savetxt("./frames/two_camera/trials/" + trialName + "/depth%d/dep%d_%d.txt" % (camNum, camNum, frameNum), img)

def ptsWriter(ptsQueue, trialName):
    while True:
        item = ptsQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        pts = nonZeroData(img)
        savePCD(img, "./frames/two_camera/trials/" + trialName + "/points%d/points%d_%d.ply" % (camNum, camNum, frameNum))

def timeWriter(destFilename, timeQueue, stopToken):
    with open(destFilename, 'w') as destFile:
        while True: 
            line = timeQueue.get()
            if line == stopToken:
                return
            destFile.write(line)


"""
Intended to take raw XYZ and return relevant
XYZ data for pointclouds

arr - 480x640x3 img
"""


def nonZeroData(arr):
    nR, nC, nL = arr.shape
    nzPts = np.array([[0,0,0]])
    for r in xrange(nR):
        row = []
        for c in xrange(nC):
            val = arr[r][c][2]
            #if val > 0.78 and val < 0.865:
            if val > 0.001:
                temp = np.array([[arr[r][c][0], arr[r][c][1], arr[r][c][2]]])
                nzPts = np.append(nzPts, temp, axis=0)
    return nzPts

"""
Takes two 3D array and makes one layer into
three 1D arrays of non-zero values

Intended to take XYZ and BGR and return as thresholded
XYZRGB data for pointclouds

colImg - 480x640x3 img
ptImg - 480x640x3 img
"""


def getThreshRGBD(colImg, ptImg):
    nR, nC, nL = ptImg.shape
    nzXYZRGB = []
    # ylayer = []
    # zlayer = []
    for r in xrange(nR):
        for c in xrange(nC):
            val = ptImg[r][c][2]
            if val > 0.78 and val < 0.865:
                nzXYZRGB.append((ptImg[r][c][0], ptImg[r][c][1], ptImg[r][c][2], colImg[r][c][2], colImg[r][c][1], colImg[r][c][0]))
                # ylayer.append()
                # zlayer.append(arr[r][c][2])
    return np.asarray(nzXYZRGB)


"""
Prints a 3d scatterplot to visualize XYZ data
"""


def make3DMap(X, Y, Z):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # surf = ax.scatter(X.flatten(), Y.flatten(), Z.flatten(), s = 20)
    surf = ax.scatter(X, Y, Z, s=20)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    plt.show()


def segDepFromGrayscale(dep, gs, thresh):
    for i in range(len(gs)):
        for j in range(len(gs[0])):
            if gs[i][j] < thresh:
                gs[i][j] = 255  # maybe don't want?
                dep[i][j] = 0

def ignoreSignals(num, stack):
    print("\nIgnoring CTRL+C and CTRL+Z \nPlease quit by focusing on window and pressing \'q\'")

"""
Class to perform all controls for realsense cameras.
This covers all streams and some integration between the hardware and image processing
"""


class RSControl:

    # depth image taken straight from ir cameras
    depth_stream = pyrs.stream.DepthStream()
    # depth image corrected to pair w/ color_stream (fixes camera offset)
    dac_stream = pyrs.stream.DACStream()
    ir_stream = pyrs.stream.InfraredStream()  # IR image
    pt_stream = pyrs.stream.PointStream()  # point image
    color_stream = pyrs.stream.ColorStream()  # rgb color image

    def __init__(self):
        self.strms = []
        self.streamColor = False
        self.streamDepth = False
        self.streamPts = False
        self.trialName = ""

    def setTrialName(self, trial):
        self.trialName = trial

    # Add color to list of camera streams
    def addColorStream(self):
        self.strms.append(self.color_stream)
        self.streamColor = True

    # add depth to list of camera streams
    def addDepStream(self):
        if len(self.strms) == 0:  # if color_stream hasn't been added
            # because throws error w/o color stream
            self.strms.append(self.color_stream)

        self.strms.append(self.depth_stream)
        self.strms.append(self.dac_stream)
        self.streamDepth = True

    def addPointStream(self):
        if len(self.strms) == 0:  # if color_stream hasn't been added
            # because throws error w/o color stream
            self.strms.append(self.color_stream)

        if len(self.strms) == 1:  # if depth streams haven't been added
            # because throws error w/o these streams
            self.strms.append(self.depth_stream)
            self.strms.append(self.dac_stream)

        self.strms.append(self.pt_stream)
        self.streamPts = True

    # create camera instance w/ color and adjusted depth streams
    # Params: saveRate - how many frames between saving to file (0 for no save)
    def startStreams(self, saveRate, nCams):
        if len(self.strms) != 0:
            with pyrs.Service() as serv:
                if nCams == 1:
                    with serv.Device(streams=self.strms) as dev:
                        dev.apply_ivcam_preset(0)
                        try:  # set custom gain/exposure values to obtain good depth image
                            custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                              (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                            dev.set_device_options(*zip(*custom_options))
                        except pyrs.RealsenseError:
                            pass  # options are not available on all devices

                        cnt = 0

                        if self.streamColor:
                            cv2.namedWindow('ColorStream')
                        if self.streamDepth:
                            cv2.namedWindow('DepthStream')
                        if self.streamPts:
                            cv2.namedWindow('PointStream')

                        while True:

                            cnt += 1
                            dev.wait_for_frames()
                            if self.streamColor:
                                color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                cv2.imshow('ColorStream', color)
                            if self.streamDepth:
                                dep = (dev.dac * dev.depth_scale)
                                cv2.imshow('DepthStream', dep)
                            if self.streamPts:
                                pts = dev.points
                                cv2.imshow('PointStream', pts)

                            # wait and check if 'q' was pressed. If so, end streams
                            keyPress = cv2.waitKey(1) & 0xFF
                            if keyPress == ord('q'):
                                break

                elif nCams == 2:
                    with serv.Device(device_id=0, streams=self.strms) as dev1, serv.Device(device_id=1, streams=self.strms) as  dev2:

                        dev1.apply_ivcam_preset(0)
                        dev2.apply_ivcam_preset(0)

                        try:  # set custom gain/exposure values to obtain good depth image
                            custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                              (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                            dev1.set_device_options(*zip(*custom_options))
                            dev2.set_device_options(*zip(*custom_options))
                        except pyrs.RealsenseError:
                            pass  # options are not available on all devices

                        cnt = 0

                        if self.streamColor:
                            cv2.namedWindow('ColorStream1')
                            cv2.namedWindow('ColorStream2')
                        if self.streamDepth:
                            cv2.namedWindow('DepthStream1')
                            cv2.namedWindow('DepthStream2')
                        if self.streamPts:
                            cv2.namedWindow('PointStream1')
                            cv2.namedWindow('PointStream2')

                        while True:

                            cnt += 1

                            dev1.wait_for_frames()
                            dev2.wait_for_frames()
                            
                            if self.streamColor:
                                color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                cv2.imshow('ColorStream1', color1)

                                color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                cv2.imshow('ColorStream2', color2)

                            if self.streamDepth:
                                dep1 = (dev1.dac * dev1.depth_scale).astype(np.float32)
                                cv2.imshow('DepthStream1', dep1)

                                dep2 = (dev2.dac * dev2.depth_scale).astype(np.float32)
                                cv2.imshow('DepthStream2', dep2)

                            if self.streamPts:
                                pts1 = dev1.points
                                pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                cv2.imshow('PointStream1', pts1)

                                pts2 = dev2.points
                                pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                cv2.imshow('PointStream2', pts2)

                            # wait and check if 'q' was pressed. If so, end streams
                            keyPress = cv2.waitKey(1) & 0xFF
                            if keyPress == ord('q'):
                                break

    def saveFeed(self, saveRate, nCams):
            cv2.namedWindow('Control Panel')
            if len(self.strms) != 0:
                signal.signal(signal.SIGINT, ignoreSignals)
                signal.signal(signal.SIGTSTP, ignoreSignals)
                with pyrs.Service() as serv:
                    if nCams == 1:
                        with serv.Device(streams=self.strms) as dev:

                            dev.apply_ivcam_preset(0)

                            try:  # set custom gain/exposure values to obtain good depth image
                                custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                                  (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                                dev.set_device_options(*zip(*custom_options))
                            except pyrs.RealsenseError:
                                pass  # options are not available on all devices

                            cnt = 0

                            f = open("./frames/two_camera/time/times.txt", "w")
                            lines = deque([])
                            colList = deque([])
                            depList = deque([])
                            d = datetime.utcnow()
                            f.write(str(d) + ", " + str(time.time()) + "\n")                            
                            while True:

                                cnt += 1

                                lines.append(str(time.time()) + "\n")
                                dev.wait_for_frames()

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    if self.streamColor:
                                        color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                        cname = "./frames/single_camera/color/frame%d.jpg" % cnt
                                        cv2.imwrite(cname, color)
                                    if self.streamDepth:
                                        np.savetxt(
                                            './frames/single_camera/depth/unproc_depvals%d.txt' % cnt, dep, fmt='%1.4f')
                                    if self.streamPts:
                                        pts = dev.points
                                        pts = nonZeroData(pts)
                                        np.savetxt('./frames/single_camera/points/nonzeroPts%d.txt' % cnt, pts, fmt = '%1.4f')

                    elif nCams == 2:
                        with serv.Device(device_id=0, streams=self.strms) as dev1, serv.Device(device_id=1, streams=self.strms) as  dev2:

                            dev1.apply_ivcam_preset(0)
                            dev2.apply_ivcam_preset(0)
                            try:  # set custom gain/exposure values to obtain good depth image
                                custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                                  (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                                dev1.set_device_options(*zip(*custom_options))
                                dev2.set_device_options(*zip(*custom_options))
                            except pyrs.RealsenseError:
                                pass  # options are not available on all devices

                            f = open("./frames/two_camera/time/times.txt", "w")
                            lines = []
                            col1List = deque([])
                            col2List = deque([])
                            dep1List = deque([])
                            dep2List = deque([])
                            pts1List = deque([])
                            pts2List = deque([])
                            d = datetime.utcnow()
                            f.write(str(d) + ", " + str(time.time()) + "\n")

                            frame = 0
                            while True:

                                lines.append(str(time.time()) + "\n")
                                dev1.wait_for_frames()
                                dev2.wait_for_frames()

                                frame += 1
                                # if saving frames is requested, save desired streams
                                if saveRate and frame % saveRate == 0:
                                    if self.streamColor:
                                        col1List.append(color1)
                                        col2List.append(color2)
                                    if self.streamDepth:
                                        dep1List.append(dep1)
                                        dep2List.append(dep2)
                                    if self.streamPts:
                                        pts1List.append(pts1)
                                        pts2List.append(pts2)

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    f.writelines(lines)
                                    ds1 = dev1.depth_scale
                                    ds2 = dev2.depth_scale
                                    cnt = 0
                                    numSaved = max(len(col1List), len(dep1List), len(pts1List))
                                    if (numSaved):
                                        for x in xrange(numSaved):
                                            cnt += 1
                                            if self.streamColor:
                                                color1 = cv2.cvtColor(col1List.popleft(), cv2.COLOR_RGB2BGR)
                                                color2 = cv2.cvtColor(col2List.popleft(), cv2.COLOR_RGB2BGR)
                                                cv2.imwrite("./frames/two_camera/color1/frame%d.jpg" % cnt, color1)
                                                cv2.imwrite("./frames/two_camera/color2/frame%d.jpg" % cnt, color2)
                                            if self.streamDepth:
                                                dep1 = dep1List.popleft() * ds1
                                                dep2 = dep2List.popleft() * ds2
                                                np.savetxt(
                                                    './frames/two_camera/depth1/dep1_%d.txt' % cnt, dep1)
                                                np.savetxt(
                                                    './frames/two_camera/depth2/dep2_%d.txt' % cnt, dep2)
                                    break
                                elif keyPress == ord('c') and saveRate == 0:
                                    if self.streamColor:
                                        col1List.append(dev1.color)
                                        col2List.append(dev2.color)

                                    if self.streamDepth:
                                        dep1List.append(dev1.dac)
                                        dep2List.append(dev2.dac)

                                    if self.streamPts:
                                        pts1 = dev1.points
                                        pts1 = nonZeroData(pts1)
                                        pts2 = dev2.points
                                        pts2 = nonZeroData(pts2)


    def startStreamAndSave(self, saveRate, nCams):
            if len(self.strms) != 0:
                signal.signal(signal.SIGINT, ignoreSignals)
                signal.signal(signal.SIGTSTP, ignoreSignals)
                with pyrs.Service() as serv:
                    if nCams == 1:
                        with serv.Device(streams=self.strms) as dev:

                            dev.apply_ivcam_preset(0)

                            try:  # set custom gain/exposure values to obtain good depth image
                                custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                                  (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                                dev.set_device_options(*zip(*custom_options))
                            except pyrs.RealsenseError:
                                pass  # options are not available on all devices
                            
                            colList = deque([])
                            depList = deque([])
                            ptsList = deque([])
                            
                            if self.streamColor:
                                cv2.namedWindow('ColorStream')                                
                            if self.streamDepth:
                                cv2.namedWindow('DepthStream')
                            if self.streamPts:
                                cv2.namedWindow('PointStream')

                            cnt = 0
                            while True:
                                cnt += 1
                                dev.wait_for_frames()

                                # Show desired streams
                                if self.streamDepth:
                                    dep = (dev.dac * dev.depth_scale)
                                    cv2.imshow('DepthStream', dep)
                                if self.streamPts:
                                    pts = dev.points
                                    # pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                    cv2.imshow('PointStream', pts)
                                if self.streamColor:
                                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                    cv2.imshow('ColorStream', color)

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    f.writelines(lines)
                                    cnt = 0
                                    numSaved = max(len(colList), len(depList), len(ptsList))
                                    if (numSaved):
                                        for x in xrange(numSaved):
                                            cnt += 1
                                            if self.streamColor:
                                                color = cv2.cvtColor(colList.popleft(), cv2.COLOR_RGB2BGR)
                                                cv2.imwrite("./frames/one_camera/trials/" + self.trialName + "/color/frame%d.jpg" % cnt, color)
                                            if self.streamDepth:
                                                dep = depList.popleft()
                                                np.savetxt(
                                                    "./frames/one_camera/trials/" + self.trialName + "/depth/dep_%d.txt" % cnt, dep)
                                            if self.streamPts:
                                                pts = nonZeroData(ptsList.popleft())
                                                savePCD(pts, "./frames/one_camera/trials/" + self.trialName + "/points/points_%d.ply" % cnt)
                                    break
                                elif keyPress == ord('c') and saveRate == 0:
                                    if self.streamColor:
                                        colList.append(color)
                                    if self.streamDepth:
                                        depList.append(dep)
                                    if self.streamPts:
                                        ptsList.append(pts)

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    if self.streamColor:
                                        colList.append(color)
                                    if self.streamDepth:
                                        depList.append(dep)
                                    if self.streamPts:
                                        ptsList.append(pts)
                    elif nCams == 2:
                        with serv.Device(device_id=0, streams=self.strms) as dev1, serv.Device(device_id=1, streams=self.strms) as  dev2:

                            dev1.apply_ivcam_preset(0)
                            dev2.apply_ivcam_preset(0)

                            try:  # set custom gain/exposure values to obtain good depth image
                                custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                                  (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                                dev1.set_device_options(*zip(*custom_options))
                                dev2.set_device_options(*zip(*custom_options))
                            except pyrs.RealsenseError:
                                pass  # options are not available on all devices

                            frame = 0
                            
                            # Implement timestamping file
                            destFile = "./frames/two_camera/trials/" + self.trialName + "/time/times.txt"
                            stopToken = "Stop Time"
                            timeList = multiprocessing.Queue()
                            timeProc = multiprocessing.Process(target = timeWriter, args=(destFile, timeList, stopToken))
                            timeProc.start()

                            d = datetime.utcnow()
                            timeList.put(str(d) + ", " + str(time.time()) + "\n")
                            
                            if self.streamColor:
                                colList = multiprocessing.Queue()
                                colProc = multiprocessing.Process(target = colWriter, args=(colList, self.trialName))
                                colProc.start()
                            if self.streamDepth:
                                dep1List = multiprocessing.Queue()
                                dep1Proc = multiprocessing.Process(target = depWriter, args=(dep1List, self.trialName))
                                dep1Proc.start()                                
                                dep2List = multiprocessing.Queue()
                                dep2Proc = multiprocessing.Process(target = depWriter, args=(dep2List, self.trialName))
                                dep2Proc.start()                                
                            if self.streamPts:
                                ptsList = multiprocessing.Queue()
                                ptsProc = multiprocessing.Process(target = ptsWriter, args=(ptsList, self.trialName))
                                ptsProc.start()


                            # Make arrays to hold images and construct viewing windows
                            if self.streamColor:
                                cv2.namedWindow('ColorStream1')
                                cv2.namedWindow('ColorStream2')
                            if self.streamDepth:
                                cv2.namedWindow('DepthStream1')
                                cv2.namedWindow('DepthStream2')
                            if self.streamPts:
                                cv2.namedWindow('PointStream1')
                                cv2.namedWindow('PointStream2')

                            while True:                            
                                dev1.wait_for_frames()
                                dev2.wait_for_frames()
                                if saveRate != 0:
                                    frame += 1
                                # Show desired streams
                                if self.streamColor:
                                    color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                    cv2.imshow('ColorStream1', color1)
                                    color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                    cv2.imshow('ColorStream2', color2)
                                if self.streamDepth:
                                    dep1 = dev1.dac * dev1.depth_scale
                                    cv2.imshow('DepthStream1', dep1)
                                    dep2 = dev2.dac * dev2.depth_scale
                                    cv2.imshow('DepthStream2', dep2)
                                if self.streamPts:
                                    pts1 = dev1.points
                                    ptStrm1 = cv2.cvtColor(pts1, cv2.COLOR_XYZ2BGR)
                                    cv2.imshow('PointStream1', ptStrm1)
                                    pts2 = dev2.points
                                    ptStrm2 = cv2.cvtColor(pts2, cv2.COLOR_XYZ2BGR)
                                    cv2.imshow('PointStream2', ptStrm2)

                                # If 'q' was pressed, save images and break
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    if self.streamColor:
                                        colList.put((color1, frame, 0))
                                        colProc.join()
                                    if self.streamDepth:
                                        dep1List.put((dep1, frame, 0))
                                        dep1Proc.join()
                                        dep2List.put((dep1, frame, 0))
                                        dep2Proc.join()
                                    if self.streamPts:
                                        ptsList.put((pts1, frame, 0))
                                        ptsProc.join()
                                    timeList.put(stopToken)
                                    timeProc.join()
                                    break
                                # If 'c' was pressed and not continuous saving, save current frames
                                elif keyPress == ord('c') and saveRate == 0:
                                    timeList.put(str(time.time()) + "\n")
                                    frame += 1
                                    if self.streamColor:
                                        colList.put((color1, frame))
                                        colList.put((color2, frame))
                                    if self.streamDepth:
                                        dep1List.put((dep1, frame))
                                        dep2List.put((dep2, frame))
                                    if self.streamPts:
                                        ptsList.put((pts1, frame))
                                        ptsList.put((pts2, frame))
                                # if saving frames is requested, save desired streams
                                if saveRate and frame % saveRate == 0:
                                    timeList.put(str(time.time()) + "\n")
                                    if self.streamColor:
                                        colList.put((color1, frame, 1))
                                        colList.put((color2, frame, 2))
                                    if self.streamDepth:
                                        dep1List.put((dep1, frame, 1))
                                        dep2List.put((dep2, frame, 2))
                                    if self.streamPts:
                                        ptsList.put((pts1, frame, 1))
                                        ptsList.put((pts2, frame, 2))
