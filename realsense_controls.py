import logging
logging.basicConfig(level=logging.INFO)

# import pcl
# import pcl.pcl_visualization

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

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


def convert_z16_to_bgr(frame):
    '''Performs depth histogram normalization
    This raw Python implementation is slow. See here for a fast implementation using Cython:
    https://github.com/pupil-labs/pupil/blob/master/pupil_src/shared_modules/cython_methods/methods.pyx
    '''
    hist = np.histogram(frame, bins=0x10000)[0]
    hist = np.cumsum(hist)
    hist -= hist[0]
    rgb_frame = np.empty(frame.shape[:2] + (3,), dtype=np.uint8)

    zeros = frame == 0
    non_zeros = frame != 0

    f = hist[frame[non_zeros]] * 255 / hist[0xFFFF]
    rgb_frame[non_zeros, 0] = 255 - f
    rgb_frame[non_zeros, 1] = 0
    rgb_frame[non_zeros, 2] = f
    rgb_frame[zeros, 0] = 20
    rgb_frame[zeros, 1] = 5
    rgb_frame[zeros, 2] = 0

    return rgb_frame


"""
Returns layers of 3D array

arr - array passed in
layer - index of desired layer
"""


def getArrayLayers(arr, layer):
    nR, nC, nL = arr.shape
    newArr = np.zeros(shape=(nR, nC))
    for r in xrange(nR):
        for c in xrange(nC):
            val = arr[r][c][layer]
            # if val >= 1:
            # print ('layer%d: '%layer + str(arr[r][c][layer]))
            if layer == 2 and val > 2.25:
                newArr[r][c] = 0
            else:
                newArr[r][c] = arr[r][c][layer]
    return newArr


"""
Takes 3D array and makes one layer into
 a 1D array of non-zero values

arr - 480x640x3 img
layer - desired layer to slice from (0, 1, or 2)
"""


def nonZeroData(arr, layer):
    nR, nC, nL = arr.shape
    newArr = []
    for r in xrange(nR):
        for c in xrange(nC):
            val = arr[r][c][layer]
            if abs(val) > 0.000001:
                if arr[r][c][2] < 2:
                    newArr.append(val)
    return newArr


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
        self.colBasDepSeg = False
        self.waterSeg = False

    """
    Add color to list of camera streams
    """

    def addColorStream(self):
        self.strms.append(self.color_stream)
        self.streamColor = True

    """
    add depth to list of camera streams
    """

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
                        last = time.time()
                        smoothing = 0.9
                        fps_smooth = 30

                        if self.streamColor:
                            cv2.namedWindow('ColorStream')
                            # cv2.namedWindow('GrayStream')
                        if self.streamDepth:
                            cv2.namedWindow('DepthStream')
                        if self.streamPts:
                            cv2.namedWindow('PointStream')
                        if self.waterSeg:
                            cv2.namedWindow('Watershed Seg')


                        while True:

                            cnt += 1
                            if (cnt % 10) == 0:
                                now = time.time()
                                dt = now - last
                                fps = 10 / dt
                                fps_smooth = (fps_smooth * smoothing) + \
                                    (fps * (1.0 - smoothing))
                                last = now

                            dev.wait_for_frames()

                            # if want to stream color images
                            if self.streamColor:
                                color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                grayscale = cv2.cvtColor(
                                    dev.color, cv2.COLOR_BGR2GRAY)
                                # cv2.namedWindow('ColorStream')
                                cv2.imshow('ColorStream', color)
                                # cv2.namedWindow('GrayStream')
                                # cv2.imshow('GrayStream', grayscale)

                                if self.waterSeg:
                                    colSeg = color
                                    # cv2.namedWindow('Watershed Seg')
                                    cv2.imshow('Watershed Seg',
                                               # imControl.watershedSegment(colSeg))
                                               imControl.watershedSegment(color))

                            # if want to stream depth images
                            if self.streamDepth:
                                dep = (dev.dac * dev.depth_scale).astype(np.float32)
                                ret, dep = cv2.threshold(
                                    dep, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                #clean depth image based on grayscale (ignore black)
                                segDepFromGrayscale(dep, cv2.cvtColor(
                                    dev.color, cv2.COLOR_RGB2GRAY), 25)

                                # cv2.namedWindow('DepthStream')
                                cv2.imshow('DepthStream', dep)

                            # if want to stream point images
                            if self.streamPts:
                                pts = dev.points
                                # pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                # cv2.namedWindow('PointStream')
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
                        last = time.time()
                        smoothing = 0.9
                        fps_smooth = 30

                        while True:

                            cnt += 1
                            if (cnt % 10) == 0:
                                now = time.time()
                                dt = now - last
                                fps = 10 / dt
                                fps_smooth = (fps_smooth * smoothing) + \
                                    (fps * (1.0 - smoothing))
                                last = now

                            dev1.wait_for_frames()
                            dev2.wait_for_frames()

                            # if want to stream color images
                            if self.streamColor:

                                color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                # grayscale1 = cv2.cvtColor(
                                #     color1, cv2.COLOR_BGR2GRAY)

                                # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                #                                   cv2.THRESH_BINARY, 17, 2)
                                cv2.namedWindow('ColorStream1')
                                cv2.imshow('ColorStream1', color1)
                                # cv2.namedWindow('GrayStream1')
                                # cv2.imshow('GrayStream1', grayscale1)

                                color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                # grayscale2 = cv2.cvtColor(
                                #     color2, cv2.COLOR_BGR2GRAY)

                                # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                #                                   cv2.THRESH_BINARY, 17, 2)
                                cv2.namedWindow('ColorStream2')
                                cv2.imshow('ColorStream2', color2)
                                # cv2.namedWindow('GrayStream2')
                                # cv2.imshow('GrayStream2', grayscale2)

                                if self.waterSeg:
                                    colSeg1 = color1
                                    cv2.namedWindow('Watershed Seg1')
                                    cv2.imshow('Watershed Seg1',
                                               imControl.watershedSegment(color1))

                                    colSeg2 = color2
                                    cv2.namedWindow('Watershed Seg2')
                                    cv2.imshow('Watershed Seg2',
                                               imControl.watershedSegment(color2))

                            # if want to stream depth images
                            if self.streamDepth:
                                dep1 = (dev1.dac * dev1.depth_scale).astype(np.float32)

                                #threshold depth to ignore beyond desired range
                                ret, dep1 = cv2.threshold(
                                    dep1, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                #clean depth image based on grayscale (ignore black)
                                # segDepFromGrayscale(dep1, cv2.cvtColor(
                                #     dev1.color, cv2.COLOR_RGB2GRAY), 25)

                                cv2.namedWindow('DepthStream1')
                                cv2.imshow('DepthStream1', dep1)

                                dep2 = (dev2.dac * dev2.depth_scale).astype(np.float32)

                                #threshold depth to ignore beyond desired range
                                ret, dep2 = cv2.threshold(
                                    dep2, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                # clean depth image based on grayscale (ignore black)
                                # segDepFromGrayscale(dep2, cv2.cvtColor(
                                #     dev1.color, cv2.COLOR_RGB2GRAY), 25)

                                cv2.namedWindow('DepthStream2')
                                cv2.imshow('DepthStream2', dep2)

                            # if want to stream point images
                            if self.streamPts:
                                pts1 = dev1.points
                                pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                cv2.namedWindow('PointStream1')
                                cv2.imshow('PointStream1', pts1)
                                pts2 = dev2.points
                                pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                cv2.namedWindow('PointStream2')
                                cv2.imshow('PointStream2', pts2)

                            # wait and check if 'q' was pressed. If so, end streams
                            keyPress = cv2.waitKey(1) & 0xFF
                            if keyPress == ord('q'):
                                break

    def saveFeed(self, saveRate, nCams):
            cv2.namedWindow('Control Panel')
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
                            last = time.time()
                            smoothing = 0.9
                            fps_smooth = 30

                            while True:

                                cnt += 1
                                if (cnt % 10) == 0:
                                    now = time.time()
                                    dt = now - last
                                    fps = 10 / dt
                                    fps_smooth = (fps_smooth * smoothing) + \
                                        (fps * (1.0 - smoothing))
                                    last = now

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

                                        # grayscale = cv2.cvtColor(
                                        #     dev.color, cv2.COLOR_BGR2GRAY)
                                        # gname = "./frames/gray/frame%d.jpg" % cnt
                                        # cv2.imwrite(gname, grayscale)

                                    if self.streamDepth:
                                        dep = (dev.dac * dev.depth_scale).astype(np.float32)
                                        ret, dep = cv2.threshold(
                                            dep, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                        # Use grayscale values to clean depth image
                                        # segDepFromGrayscale(dep, cv2.cvtColor(
                                        #     dev.color, cv2.COLOR_RGB2GRAY), 25)

                                        dname = "./frames/single_camera/depth/frame%d.jpg" % cnt
                                        cv2.imwrite(dname, dep)
                                        ret, dTest = cv2.threshold(
                                            dep, 2, 10, cv2.THRESH_TOZERO_INV)
                                        cv2.imwrite(dname, dTest)

                                        np.savetxt(
                                            './frames/single_camera/depth/thresh_depvals%d.txt' % cnt, dTest)
                                        np.savetxt(
                                            './frames/single_camera/depth/unproc_depvals%d.txt' % cnt, dTest)

                                        if self.colBasDepSeg:
                                            gname = "./frames/single_camera/gray/frame%d.jpg" % cnt
                                            grayscale = cv2.cvtColor(
                                                dev.color, cv2.COLOR_RGB2GRAY)
                                            cv2.imwrite(gname, grayscale)

                                    if self.streamPts:
                                        pts = dev.points
                                        ptname = "./frames/single_camera/points/frame%d.jpeg" % cnt
                                        cv2.imwrite(ptname, pts)
                                        # xlayer = getArrayLayers(pts, 0)
                                        # ylayer = getArrayLayers(pts, 1)
                                        # zlayer = getArrayLayers(pts, 2)
                                        # xlayer = nonZeroData(pts, 0)
                                        # ylayer = nonZeroData(pts, 1)
                                        # zlayer = nonZeroData(pts, 2)
                                        # np.savetxt('./frames/points/xframe_vals%d.txt' % cnt, xlayer, fmt = '%.2f')
                                        # np.savetxt('./frames/points/yframe_vals%d.txt' % cnt, ylayer, fmt = '%.2f')
                                        # np.savetxt('./frames/points/zframe_vals%d.txt' % cnt, zlayer, fmt = '%.2f')
                                        # make3DMap(xlayer, ylayer, zlayer)

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
                            last = time.time()
                            smoothing = 0.9
                            fps_smooth = 30

                            while True:

                                cnt += 1
                                if (cnt % 10) == 0:
                                    now = time.time()
                                    dt = now - last
                                    fps = 10 / dt
                                    fps_smooth = (fps_smooth * smoothing) + \
                                        (fps * (1.0 - smoothing))
                                    last = now

                                dev1.wait_for_frames()
                                dev2.wait_for_frames()

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    for x in xrange(1, nCams+1):
                                        if self.streamColor:
                                            color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                            color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                            cname1 = "./frames/two_camera/color1/frame%d.jpg" % cnt
                                            cv2.imwrite(cname1, color1)
                                            cname2 = "./frames/two_camera/color2/frame%d.jpg" % cnt
                                            cv2.imwrite(cname2, color2)

                                        if self.streamDepth:
                                            dep1 = dev1.dac * dev1.depth_scale
                                            dep2 = dev2.dac * dev2.depth_scale

                                            dname1 = "./frames/two_camera/depth1/frame%d.jpg" % cnt
                                            cv2.imwrite(dname1, dep1)

                                            # ret, dTest1 = cv2.threshold(
                                            #     dep1, 2, 10, cv2.THRESH_TOZERO_INV)
                                            # cv2.imwrite(dname, dTest1)

                                            # np.savetxt(
                                            #     './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                            # np.savetxt(
                                            #     './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                            dname2 = "./frames/two_camera/depth2/frame%d.jpg" % cnt
                                            cv2.imwrite(dname2, dep2)

                                            # threshold to only track depth in certain range
                                            # ret, dTest1 = cv2.threshold(
                                            #     dep1, 2, 10, cv2.THRESH_TOZERO_INV)
                                            # cv2.imwrite(dname2, dTest2)

                                            # np.savetxt(
                                            #     './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                            # np.savetxt(
                                            #     './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                        if self.streamPts:
                                            ptname1 = "./frames/two_camera/points1/frame%d.jpeg" % cnt
                                            cv2.imwrite(ptname1, pts1)
                                            ptname2 = "./frames/two_camera/points2/frame%d.jpeg" % cnt
                                            cv2.imwrite(ptname2, pts2)
                                            # xlayer = getArrayLayers(pts, 0)
                                            # ylayer = getArrayLayers(pts, 1)
                                            # zlayer = getArrayLayers(pts, 2)
                                            # xlayer = nonZeroData(pts1, 0)
                                            # ylayer = nonZeroData(pts1, 1)
                                            # zlayer = nonZeroData(pts1, 2)
                                            # np.savetxt('./frames/points/xframe_vals%d.txt' % cnt, xlayer, fmt = '%.2f')
                                            # np.savetxt('./frames/points/yframe_vals%d.txt' % cnt, ylayer, fmt = '%.2f')
                                            # np.savetxt('./frames/points/zframe_vals%d.txt' % cnt, zlayer, fmt = '%.2f')
                                            # make3DMap(xlayer, ylayer, zlayer)


    def startStreamAndSave(self, saveRate, nCams):
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
                            last = time.time()
                            smoothing = 0.9
                            fps_smooth = 30

                            while True:

                                cnt += 1
                                if (cnt % 10) == 0:
                                    now = time.time()
                                    dt = now - last
                                    fps = 10 / dt
                                    fps_smooth = (fps_smooth * smoothing) + \
                                        (fps * (1.0 - smoothing))
                                    last = now

                                dev.wait_for_frames()

                                # if want to stream color images
                                if self.streamColor:

                                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                    grayscale = cv2.cvtColor(
                                        dev.color, cv2.COLOR_BGR2GRAY)

                                    # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    #                                   cv2.THRESH_BINARY, 17, 2)
                                    cv2.namedWindow('ColorStream')
                                    cv2.imshow('ColorStream', color)
                                    cv2.namedWindow('GrayStream')
                                    cv2.imshow('GrayStream', grayscale)

                                    if self.waterSeg:
                                        colSeg = color
                                        # imControl.threshColImg(colSeg)
                                        cv2.namedWindow('Watershed Seg')
                                        cv2.imshow('Watershed Seg',
                                                   # imControl.watershedSegment(colSeg))
                                                   imControl.watershedSegment(color))

                                # if want to stream depth images
                                if self.streamDepth:
                                    dep = (dev.dac * dev.depth_scale).astype(np.float32)
                                    ret, dep = cv2.threshold(
                                        dep, 1.7, 10, cv2.THRESH_TOZERO_INV)
                                    segDepFromGrayscale(dep, cv2.cvtColor(
                                        dev.color, cv2.COLOR_RGB2GRAY), 25)
                                    cv2.namedWindow('DepthStream')
                                    cv2.imshow('DepthStream', dep)

                                # if want to stream point images
                                if self.streamPts:
                                    pts = dev.points
                                    # pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream')
                                    cv2.imshow('PointStream', pts)

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    if self.streamDepth:
                                        dname = "./frames/depth/frame%d.jpg" % cnt
                                        cv2.imwrite(dname, dep)
                                        ret, dTest = cv2.threshold(
                                            dep, 2, 10, cv2.THRESH_TOZERO_INV)
                                        cv2.imwrite(dname, dTest)

                                        np.savetxt(
                                            './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                        np.savetxt(
                                            './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                        if self.colBasDepSeg:
                                            gname = "./frames/gray/frame%d.jpg" % cnt
                                            grayscale = cv2.cvtColor(
                                                dev.color, cv2.COLOR_RGB2GRAY)
                                            cv2.imwrite(gname, grayscale)

                                    if self.streamColor:
                                        cname = "./frames/color/frame%d.jpg" % cnt
                                        gname = "./frames/gray/frame%d.jpg" % cnt
                                        cv2.imwrite(cname, color)
                                        cv2.imwrite(gname, grayscale)

                                    if self.streamPts:
                                        ptname = "./frames/points/frame%d.jpeg" % cnt
                                        cv2.imwrite(ptname, pts)
                                        # xlayer = getArrayLayers(pts, 0)
                                        # ylayer = getArrayLayers(pts, 1)
                                        # zlayer = getArrayLayers(pts, 2)
                                        xlayer = nonZeroData(pts, 0)
                                        ylayer = nonZeroData(pts, 1)
                                        zlayer = nonZeroData(pts, 2)
                                        # np.savetxt('./frames/points/xframe_vals%d.txt' % cnt, xlayer, fmt = '%.2f')
                                        # np.savetxt('./frames/points/yframe_vals%d.txt' % cnt, ylayer, fmt = '%.2f')
                                        # np.savetxt('./frames/points/zframe_vals%d.txt' % cnt, zlayer, fmt = '%.2f')
                                        # make3DMap(xlayer, ylayer, zlayer)

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
                            last = time.time()
                            smoothing = 0.9
                            fps_smooth = 30

                            while True:

                                cnt += 1
                                if (cnt % 10) == 0:
                                    now = time.time()
                                    dt = now - last
                                    fps = 10 / dt
                                    fps_smooth = (fps_smooth * smoothing) + \
                                        (fps * (1.0 - smoothing))
                                    last = now

                                dev1.wait_for_frames()
                                dev2.wait_for_frames()

                                # if want to stream color images
                                if self.streamColor:

                                    color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                    grayscale1 = cv2.cvtColor(
                                        color1, cv2.COLOR_BGR2GRAY)

                                    # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    #                                   cv2.THRESH_BINARY, 17, 2)
                                    cv2.namedWindow('ColorStream1')
                                    cv2.imshow('ColorStream1', color1)
                                    cv2.namedWindow('GrayStream1')
                                    cv2.imshow('GrayStream1', grayscale1)

                                    color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                    grayscale2 = cv2.cvtColor(
                                        color2, cv2.COLOR_BGR2GRAY)

                                    # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    #                                   cv2.THRESH_BINARY, 17, 2)
                                    cv2.namedWindow('ColorStream2')
                                    cv2.imshow('ColorStream2', color2)
                                    cv2.namedWindow('GrayStream2')
                                    cv2.imshow('GrayStream2', grayscale2)

                                    if self.waterSeg:
                                        colSeg1 = color1
                                        cv2.namedWindow('Watershed Seg1')
                                        cv2.imshow('Watershed Seg1',
                                                   imControl.watershedSegment(color1))

                                        colSeg2 = color2
                                        cv2.namedWindow('Watershed Seg2')
                                        cv2.imshow('Watershed Seg2',
                                                   imControl.watershedSegment(color2))

                                # if want to stream depth images
                                if self.streamDepth:
                                    dep1 = dev1.dac * dev1.depth_scale

                                    #threshold depth to ignore beyond desired range
                                    ret, dep1 = cv2.threshold(
                                        dep1, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                    #clean depth image based on grayscale (ignore black)
                                    segDepFromGrayscale(dep1, cv2.cvtColor(
                                        dev1.color, cv2.COLOR_RGB2GRAY), 25)

                                    cv2.namedWindow('DepthStream1')
                                    cv2.imshow('DepthStream1', dep1)

                                    dep2 = dev2.dac * dev2.depth_scale

                                    #threshold depth to ignore beyond desired range
                                    ret, dep2 = cv2.threshold(
                                        dep2, 1.7, 10, cv2.THRESH_TOZERO_INV)

                                    #clean depth image based on grayscale (ignore black)
                                    segDepFromGrayscale(dep2, cv2.cvtColor(
                                        dev1.color, cv2.COLOR_RGB2GRAY), 25)

                                    cv2.namedWindow('DepthStream2')
                                    cv2.imshow('DepthStream2', dep2)

                                # if want to stream point images
                                if self.streamPts:
                                    pts1 = dev1.points
                                    pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream1')
                                    cv2.imshow('PointStream1', pts1)
                                    pts2 = dev2.points
                                    pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream2')
                                    cv2.imshow('PointStream2', pts2)

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    for x in xrange(1, nCams+1):
                                        if self.streamDepth:
                                            dname1 = "./frames/depth1/frame%d.jpg" % cnt
                                            cv2.imwrite(dname1, dep1)
                                            ret, dTest1 = cv2.threshold(
                                                dep1, 2, 10, cv2.THRESH_TOZERO_INV)
                                            cv2.imwrite(dname, dTest1)

                                            np.savetxt(
                                                './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                            np.savetxt(
                                                './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                            dname2 = "./frames/depth2/frame%d.jpg" % cnt
                                            cv2.imwrite(dname2, dep2)
                                            ret, dTest1 = cv2.threshold(
                                                dep1, 2, 10, cv2.THRESH_TOZERO_INV)
                                            cv2.imwrite(dname2, dTest2)

                                            np.savetxt(
                                                './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                            np.savetxt(
                                                './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                            if self.colBasDepSeg:
                                                gname1 = "./frames/gray1/frame%d.jpg" % cnt
                                                grayscale1 = cv2.cvtColor(
                                                    dev1.color, cv2.COLOR_RGB2GRAY)
                                                cv2.imwrite(gname1, grayscale1)
                                                gname2 = "./frames/gray2/frame%d.jpg" % cnt
                                                grayscale2 = cv2.cvtColor(
                                                    dev2.color, cv2.COLOR_RGB2GRAY)
                                                cv2.imwrite(gname2, grayscale2)


                                            if self.colBasDepSeg:
                                                gname1 = "./frames/gray1/frame%d.jpg" % cnt
                                                grayscale1 = cv2.cvtColor(
                                                    dev1.color, cv2.COLOR_RGB2GRAY)
                                                cv2.imwrite(gname1, grayscale1)

                                        if self.streamColor:
                                            cname1 = "./frames/color1/frame%d.jpg" % cnt
                                            gname1 = "./frames/gray1/frame%d.jpg" % cnt
                                            cv2.imwrite(cname1, color1)
                                            cv2.imwrite(gname1, grayscale1)
                                            cname2 = "./frames/color2/frame%d.jpg" % cnt
                                            gname2 = "./frames/gray2/frame%d.jpg" % cnt
                                            cv2.imwrite(cname2, color2)
                                            cv2.imwrite(gname2, grayscale2)

                                        if self.streamPts:
                                            ptname1 = "./frames/points1/frame%d.jpeg" % cnt
                                            cv2.imwrite(ptname1, pts1)
                                            ptname2 = "./frames/points2/frame%d.jpeg" % cnt
                                            cv2.imwrite(ptname2, pts2)
                                            # xlayer = getArrayLayers(pts, 0)
                                            # ylayer = getArrayLayers(pts, 1)
                                            # zlayer = getArrayLayers(pts, 2)
                                            # xlayer = nonZeroData(pts1, 0)
                                            # ylayer = nonZeroData(pts1, 1)
                                            # zlayer = nonZeroData(pts1, 2)
                                            # np.savetxt('./frames/points/xframe_vals%d.txt' % cnt, xlayer, fmt = '%.2f')
                                            # np.savetxt('./frames/points/yframe_vals%d.txt' % cnt, ylayer, fmt = '%.2f')
                                            # np.savetxt('./frames/points/zframe_vals%d.txt' % cnt, zlayer, fmt = '%.2f')
                                            # make3DMap(xlayer, ylayer, zlayer)