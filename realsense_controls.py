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
import socket

# Saves array of XYZ data to pointcloud
# Performs some filtering first
def savePCD(arr, fnameToWrite):
    p = pcl.PointCloud()
    p.from_array(np.array(arr,  dtype=np.float32))
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (10)
    fil.set_std_dev_mul_thresh (1.0)
    res = fil.filter()
    pcl.save(p, fnameToWrite)

# Functions for multiprocessing of data writing
# Each function writes separate data stream
def colWriter(colQueue, trialName):
    while True:
        item = colQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        elif camNum == 1 or camNum == 2:
            cv2.imwrite("./frames/two_camera/trials/" + trialName + "/color%d/frame%d.jpg" % (camNum, frameNum), img)
        elif camNum == 3:
            cv2.imwrite("./frames/one_camera/trials/" + trialName + "/color/frame%d.jpg" % frameNum, img)

def depWriter(depQueue, trialName):
    while True:
        item = depQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        elif camNum == 1 or camNum == 2:
            np.savetxt("./frames/two_camera/trials/" + trialName + "/depth%d/dep%d_%d.txt" % (camNum, camNum, frameNum), img)
        elif camNum == 3:
            np.savetxt("./frames/one_camera/trials/" + trialName + "/depth/dep_%d.txt" % frameNum, img)

def ptsWriter(ptsQueue, trialName):
    while True:
        item = ptsQueue.get()
        img = item[0]
        frameNum = item[1]
        camNum = item[2]
        if camNum == 0:
            return
        pts = nonZeroData(img)
        elif camNum == 1 or camNum == 2:
            savePCD(img, "./frames/two_camera/trials/" + trialName + "/points%d/points%d_%d.ply" % (camNum, camNum, frameNum))
        elif camNum == 3:
            savePCD(img, "./frames/one_camera/trials/" + trialName + "/points/points_%d.ply" % frameNum)

def timeWriter(destFilename, timeQueue, stopToken):
    with open(destFilename, 'w') as destFile:
        while True: 
            line = timeQueue.get()
            if line == stopToken:
                return
            destFile.write(line)

"""
Intended to take raw XYZ and return
XYZ data with non-zero Z values for pointclouds

arr - 480x640x3 img
output - N x 3 list of points
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

# signal ignore added to protect from cancelling before all stream data is written
def ignoreSignals(num, stack):
    print("\nIgnoring CTRL+C and CTRL+Z \nPlease quit by focusing on window and pressing \'q\'")
    return

"""
Class to perform all controls for realsense cameras.
This covers all streams and some integration between the hardware and image processing
"""
class RSControl:
    depth_stream = pyrs.stream.DepthStream() # depth image
    dac_stream = pyrs.stream.DACStream()        # depth-adjusted-to-color
    ir_stream = pyrs.stream.InfraredStream()  # IR image
    pt_stream = pyrs.stream.PointStream()  # point image
    color_stream = pyrs.stream.ColorStream()  # rgb color image

    def __init__(self):
        self.strms = [] # list of active streams
        self.trialName = "" # name of current trial to save data under

        # booleans to track what streams are active
        self.streamColor = False
        self.streamDepth = False
        self.streamPts = False

    def setTrialName(self, trial):
        self.trialName = trial

    def addColorStream(self):
        self.strms.append(self.color_stream)
        self.streamColor = True

    def addDepStream(self):
        # because throws error w/o color stream
        if len(self.strms) == 0:
            self.strms.append(self.color_stream)

        self.strms.append(self.depth_stream)
        self.strms.append(self.dac_stream)
        self.streamDepth = True

    def addPointStream(self):
        # because throws error w/o color stream
        if len(self.strms) == 0:
            self.strms.append(self.color_stream)
        # because throws error w/o these streams
        if len(self.strms) == 1: 
            self.strms.append(self.depth_stream)
            self.strms.append(self.dac_stream)

        self.strms.append(self.pt_stream)
        self.streamPts = True

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
                            
                            # Implement timestamping file
                            timeDestFile = "./frames/one_camera/trials/" + self.trialName + "/time/times.txt"
                            stopToken = "Stop Time"
                            timeList = multiprocessing.Queue()
                            timeProc = multiprocessing.Process(target = timeWriter, args=(timeDestFile, timeList, stopToken))
                            timeProc.start()
                            d = datetime.utcnow()
                            timeList.put(str(d) + ", " + str(time.time()) + "\n")
                            
                            if self.streamColor:
                                colList = multiprocessing.Queue()
                                colProc = multiprocessing.Process(target = colWriter, args=(colList, self.trialName))
                                colProc.start()
                            if self.streamDepth:
                                depList = multiprocessing.Queue()
                                depProc = multiprocessing.Process(target = depWriter, args=(dep1List, self.trialName))
                                depProc.start()                               
                            if self.streamPts:
                                ptsList = multiprocessing.Queue()
                                ptsProc = multiprocessing.Process(target = ptsWriter, args=(ptsList, self.trialName))
                                ptsProc.start()

                            # Construct viewing windows
                            if self.streamColor:
                                cv2.namedWindow('ColorStream')
                            if self.streamDepth:
                                cv2.namedWindow('DepthStream')
                            if self.streamPts:
                                cv2.namedWindow('PointStream')

                            # Make socket to listen to other computer
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                            # Hardcoded values, were personally set
                            s.bind(('10.0.0.60', 12345))
                            s.listen(5)
                            s.setblocking(False)

                            frame = 0
                            while True:

                                # Set up to control streaming by socket communication
                                # Not fully debugged yet                                
                                streaming = 0
                                try:
                                    conn, addr = s.accept()
                                    streaming = int(conn.recv(4096))
                                except socket.error as e:
                                    continue
                                print streaming

                                if streaming:                            
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
                                    # If 'c' was pressed and not continuous saving, save current frames
                                    # if continuous saving, save desired streams at desired rate
                                    keyPress = cv2.waitKey(1) & 0xFF
                                    if keyPress == ord('q'):
                                        if self.streamColor:
                                            colList.put((color, frame, 0))
                                            colProc.join()
                                        if self.streamDepth:
                                            dep1List.put((dep, frame, 0))
                                            dep1Proc.join()
                                        if self.streamPts:
                                            ptsList.put((pts, frame, 0))
                                            ptsProc.join()
                                        timeList.put(stopToken)
                                        timeProc.join()
                                        break
                                    elif keyPress == ord('c') and saveRate == 0:
                                        timeList.put(str(time.time()) + "\n")
                                        frame += 1
                                        if self.streamColor:
                                            colList.put((color, frame, 3))
                                        if self.streamDepth:
                                            dep1List.put((dep, frame, 3))
                                        if self.streamPts:
                                            ptsList.put((pts, frame, 3))
                                    elif saveRate and frame % saveRate == 0:
                                        timeList.put(str(time.time()) + "\n")
                                        if self.streamColor:
                                            colList.put((color, frame, 3))
                                        if self.streamDepth:
                                            dep1List.put((dep, frame, 3))
                                        if self.streamPts:
                                            ptsList.put((pts, frame, 3))
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

                            # Construct viewing windows
                            if self.streamColor:
                                cv2.namedWindow('ColorStream1')
                                cv2.namedWindow('ColorStream2')
                            if self.streamDepth:
                                cv2.namedWindow('DepthStream1')
                                cv2.namedWindow('DepthStream2')
                            if self.streamPts:
                                cv2.namedWindow('PointStream1')
                                cv2.namedWindow('PointStream2')

                            # Make socket to listen to other computer
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.bind(('10.0.0.60', 12345))
                            s.listen(5)
                            s.setblocking(False)

                            frame = 0
                            while True:
                                # Set up to control streaming by socket communication
                                # Not fully debugged yet                                
                                streaming = 0
                                try:
                                    conn, addr = s.accept()
                                    streaming = int(conn.recv(4096))
                                except socket.error as e:
                                    continue
                                print streaming

                                if streaming:                            
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
                                    # If 'c' was pressed and not continuous saving, save current frames
                                    # if saving frames is requested, save desired streams
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
