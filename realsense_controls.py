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
import pcl

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

def transformPCD(arr, T):
    pcd = np.array([[0] * 3] * len(arr), dtype=np.float32)
    for i in range(0, len(arr)):
        pcd[i] = np.matmul(T, (list(arr[i]) + [1]))[0:3]
    return pcd

def saveTransformedPCD(arr, fnameToWrite):
    p = pcl.PointCloud(arr)
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (10)
    fil.set_std_dev_mul_thresh (1.0)
    res = fil.filter()
    pcl.save(res, fnameToWrite)

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
three 1D arrays of non-zero values

Intended to take XYZ and return as thresholded
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
        self.findCorners = False
        self.XYZRGB = False
        self.cam1_x1 = 290
        self.cam1_x2 = 405
        self.cam1_y1 = 100
        self.cam1_y2 = 180
        self.cam2_x1 = 326
        self.cam2_x2 = 385
        self.cam2_y1 = 155
        self.cam2_y2 = 300


    """
    Add color to list of camera streams
    """

    def addColorStream(self):
        self.strms.append(self.color_stream)
        self.streamColor = True


    def setCropping(self, cam, x1, x2, y1, y2):
        if cam == 1:
            self.cam1_x1 = x1
            self.cam1_x2 = x2
            self.cam1_y1 = y1
            self.cam1_y2 = y2
        elif cam == 2:
            self.cam2_x1 = x1
            self.cam2_x2 = x2
            self.cam2_y1 = y1
            self.cam2_y2 = y2

    """
    Turn on RGBD imaging
    """

    def setXYZRGB(self):
        self.XYZRGB = True

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
                            
                            while True:

                                cnt += 1
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

                            cnt = 0

                            while True:

                                cnt += 1

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
                                            cv2.imwrite("./frames/two_camera/color1/frame%d.jpg" % cnt, color1)
                                            cv2.imwrite("./frames/two_camera/color2/frame%d.jpg" % cnt, color2)

                                        if self.streamDepth:
                                            dep1 = dev1.dac * dev1.depth_scale
                                            dep2 = dev2.dac * dev2.depth_scale

                                            cv2.imwrite("./frames/two_camera/depth1/frame%d.jpg" % cnt, dep1)

                                            # np.savetxt(
                                            #     './frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                                            # np.savetxt(
                                            #     './frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                                            dname2 = "./frames/two_camera/depth2/frame%d.jpg" % cnt
                                            cv2.imwrite(dname2, dep2)


                                        if self.streamPts:
                                            T_top_to_side = np.array([[0.08, 0.99, 0.15, 0.11], [-0.29, -0.12, 0.95, -0.76], [0.95, -0.12, 0.27, 0.6], [0, 0, 0, 1]])
                                            pts1 = dev1.points
                                            pts1 = nonZeroData(pts1)
                                            b = np.ones((len(pts1), 4))
                                            b[:,:-1] = pts1
                                            pts1 = transformPCD(b, T_top_to_side)
                                            #np.savetxt('./frames/two_camera/points/nonzeroPts1_%d.txt' % cnt, pts1, fmt = '%1.4f')
                                            pts2 = dev2.points
                                            pts2 = nonZeroData(pts2)
                                            b = np.ones((len(pts2), 4))
                                            b[:,:-1] = pts2
                                            pts2 = transformPCD(b, np.eye(4))
                                            combined_pts = np.concat(pts1, pts2)
                                            saveTransformedPLY(combined_pts, './frames/two_camera/points/merged_points_%d.ply')

                                            #np.savetxt('./frames/two_camera/points/nonzeroPts2_%d.txt' % cnt, pts2, fmt = '%1.4f')




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
                            while True:

                                cnt += 1

                                dev.wait_for_frames()


                                # if want to stream depth images
                                if self.streamDepth:
                                    dep = (dev.dac * dev.depth_scale).astype(np.float32)
                                    ret, dep = cv2.threshold(
                                        dep, 0.88, 10, cv2.THRESH_TOZERO_INV)
                                    cv2.namedWindow('DepthStream')
                                    cv2.imshow('DepthStream', dep)

                                # if want to stream point images
                                if self.streamPts:
                                    pts = dev.points
                                    # pts = cv2.cvtColor(pts, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream')
                                    cv2.imshow('PointStream', pts)

                                # if want to stream color images
                                if self.streamColor:

                                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                                    color = color[self.cam1_y1:self.cam1_y2, self.cam1_x1:self.cam1_x2]

                                    if self.streamDepth:
                                        # Eadom look here to control color segmentation
                                        imControl.segColFromDepth(color, dep, 0.88, True)
                                        imControl.segColFromDepth(color, dep, 0.65, False)


                                    cv2.namedWindow('ColorStream')
                                    cv2.imshow('ColorStream', color)


                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break
                                # if keyPress == ord('c'):
                                #     np.savetxt(
                                #         './frames/single_camera/depth/unproc_depvals%d.txt' % cnt, dep, newline=' ', fmt='%1.4f')
                                #     cname = "./frames/single_camera/color/frame%d.png" % cnt
                                #     cv2.imwrite(cname, color)


                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    if self.streamDepth:
                                        depCrop = dep[self.cam1_y1:self.cam1_y2, self.cam1_x1:self.cam1_x2]
                                        np.savetxt('./frames/single_camera/depth/crop_depvals%d.txt' % cnt, depCrop,
                                            header=str(self.cam1_x2-self.cam1_x1)+" "+str(self.cam1_y2-self.cam1_y1)+"\n", comments='', newline=' ', fmt='%1.4f')
                                        # np.savetxt('./frames/single_camera/depth/unproc_depvals%d.txt' % cnt, dep, newline=' ', fmt='%1.4f')


                                    if self.streamColor:
                                        cname = "./frames/single_camera/color/frame%d.png" % cnt
                                        cv2.imwrite(cname, color)

                                    if self.streamPts:
                                            pts1 = dev1.points
                                            pts1 = nonZeroData(pts1)
                                            np.savetxt('./frames/single_camera/points/nonzeroPts%d.txt' % cnt, pts1, fmt = '%1.4f')


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

                            while True:

                                cnt += 1
                                
                                dev1.wait_for_frames()
                                dev2.wait_for_frames()

                                # if want to stream color images
                                if self.streamColor:

                                    color1 = cv2.cvtColor(dev1.color, cv2.COLOR_RGB2BGR)
                                    cropColor1 = color1[self.cam1_y1:self.cam1_y2, self.cam1_x1:self.cam1_x2]

                                    cv2.namedWindow('ColorStream1')
                                    cv2.imshow('ColorStream1', color1)
                                    cv2.namedWindow('CroppedColor1')
                                    cv2.imshow('CroppedColor1', cropColor1)

                                    color2 = cv2.cvtColor(dev2.color, cv2.COLOR_RGB2BGR)
                                    cropColor2 = color2[self.cam2_y1:self.cam2_y2, self.cam2_x1:self.cam2_x2]

                                    cv2.namedWindow('ColorStream2')
                                    cv2.imshow('ColorStream2', color2)
                                    cv2.namedWindow('CroppedColor2')
                                    cv2.imshow('CroppedColor2', cropColor2)


                                # if want to stream depth images
                                if self.streamDepth:
                                    dep1 = dev1.dac * dev1.depth_scale
                                    depCrop1 = dep1[self.cam1_y1:self.cam1_y2, self.cam1_x1:self.cam1_x2]
                                
                                    cv2.namedWindow('DepthStream1')
                                    cv2.imshow('DepthStream1', depCrop1)

                                    dep2 = dev2.dac * dev2.depth_scale
                                    depCrop2 = dep2[self.cam2_y1:self.cam2_y2, self.cam2_x1:self.cam2_x2]

                                    cv2.namedWindow('DepthStream2')
                                    cv2.imshow('DepthStream2', depCrop2)

                                # if want to stream point images
                                if self.streamPts:
                                    pts1 = dev1.points
                                    ptStrm1 = cv2.cvtColor(pts1, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream1')
                                    cv2.imshow('PointStream1', ptStrm1)
                                    pts2 = dev2.points
                                    ptStrm2 = cv2.cvtColor(pts2, cv2.COLOR_XYZ2BGR)
                                    cv2.namedWindow('PointStream2')
                                    cv2.imshow('PointStream2', ptStrm2)

                                # wait and check if 'q' was pressed. If so, end streams
                                keyPress = cv2.waitKey(1) & 0xFF
                                if keyPress == ord('q'):
                                    break
                                elif keyPress == ord('c'):
                                    if self.streamDepth:
                                        np.savetxt(
                                            './frames/two_camera/depth1/dep1_%d.txt' % cnt, dep1)
                                        np.savetxt(
                                            './frames/two_camera/depth2/dep2_%d.txt' % cnt, dep2)

                                    if self.streamColor:
                                        cname1 = "./frames/two_camera/color1/frame%d.jpg" % cnt
                                        cv2.imwrite(cname1, color1)
                                        cname2 = "./frames/two_camera/color2/frame%d.jpg" % cnt
                                        cv2.imwrite(cname2, color2)

                                # if saving frames is requested, save desired streams
                                if saveRate and cnt % saveRate == 0:
                                    for x in xrange(1, nCams+1):
                                        if self.streamDepth:
                                            np.savetxt('./frames/two_camera/depth1/crop_depvals1_%d.txt' % cnt, depCrop1,
                                                header=str(self.cam1_x2-self.cam1_x1)+" "+str(self.cam1_y2-self.cam1_y1)+"\n", comments='', newline=' ', fmt='%1.4f')
                                            np.savetxt('./frames/two_camera/depth2/crop_depvals2_%d.txt' % cnt, depCrop2,
                                                header=str(self.cam2_x2-self.cam2_x1)+" "+str(self.cam2_y2-self.cam2_y1)+"\n", comments='', newline=' ', fmt='%1.4f')

                                            # np.savetxt(
                                            #     './frames/two_camera/depth1/dep1_%d.txt' % cnt, dep1)
                                            # np.savetxt(
                                            #     './frames/two_camera/depth2/dep2_%d.txt' % cnt, dep2)

                                        if self.streamColor:
                                            cname1 = "./frames/two_camera/color1/frame%d.jpg" % cnt
                                            cv2.imwrite(cname1, color1)
                                            cname2 = "./frames/two_camera/color2/frame%d.jpg" % cnt
                                            cv2.imwrite(cname2, color2)

                                        if self.streamPts:
                                            T_top_to_side = np.array([[0.08, 0.99, 0.15, 0.11], [-0.29, -0.12, 0.95, -0.76], [0.95, -0.12, 0.27, 0.6], [0, 0, 0, 1]])
                                            nzPts1 = nonZeroData(pts1)
                                            print nzPts1.shape
                                            pcdPts1 = transformPCD(nzPts1, T_top_to_side)
                                            #np.savetxt('./frames/two_camera/points/nonzeroPts1_%d.txt' % cnt, pts1, fmt = '%1.4f')
                                            nzPts2 = nonZeroData(pts2)
                                            pcdPts2 = transformPCD(nzPts2, np.eye(4))
                                            combined_pts = np.concatenate((pcdPts1, pcdPts2))
                                            #np.savetxt('./frames/two_camera/points/nonzeroPts2_%d.txt' % cnt, pts2, fmt = '%1.4f')
                                            saveTransformedPCD(combined_pts, './frames/two_camera/merged_points/merged_points_%d.ply'%cnt)
