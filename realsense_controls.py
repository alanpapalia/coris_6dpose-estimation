import logging
logging.basicConfig(level=logging.INFO)

import os, shutil
import time
import numpy as np
import scipy
from scipy import misc
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option
from pyrealsense import offline





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

class RSControl:

    depth_stream = pyrs.stream.DepthStream()  # depth image taken straight from ir cameras
    dac_stream = pyrs.stream.DACStream()  # depth image corrected to pair w/ color_stream (fixes camera offset)
    ir_stream = pyrs.stream.InfraredStream()  # IR image
    pt_stream = pyrs.stream.PointStream()  # point image
    color_stream = pyrs.stream.ColorStream()  # rgb color image

    def __init__(self):
        self.strms = []
        self.streamColor = False
        self.streamDepth = False
        self.streamPts = False
        self.streamIR = False


    # add depth stream to camera
    def addDepStream(self):
        self.strms.append(self.depth_stream)
        self.strms.append(self.dac_stream)
        self.strms.append(self.color_stream)
        self.streamDepth = True

    # add depth stream to camera
    def addColorStream(self):
        self.strms.append(self.color_stream)
        self.streamColor = True

    def addPointStream(self):
        self.strms.append(self.pt_stream)
        self.streamPts = True

    def addIRStream(self):
        self.strms.append(self.ir_stream)
        self.streamIR = True

    # create camera instance w/ color and adjusted depth streams
    # Params: saveRate - how many frames between saving to file (0 for no save)
    def startStreams(self, saveRate):
        with pyrs.Service() as serv:
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
                        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0 - smoothing))
                        last = now

                    dev.wait_for_frames()

                    #if want to stream color images
                    if self.streamColor:

                        color = dev.color
                        grayscale = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
                        color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

                        # grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        #                                   cv2.THRESH_BINARY, 17, 2)

                        cv2.namedWindow('ColorStream')
                        cv2.imshow('ColorStream', color)
                        cv2.namedWindow('GrayStream')
                        cv2.imshow('GrayStream', grayscale)

                    #if want to stream depth images
                    if self.streamDepth:
                        dep = dev.dac * dev.depth_scale
                        cv2.namedWindow('DepthStream')
                        cv2.imshow('DepthStream', dep)

                    #if want to stream point images
                    if self.streamPts:
                        pts = dev.points
                        cv2.namedWindow('PointStream')
                        cv2.imshow('PointStream', pts)

                    #if want to stream IR images
                    if self.streamIR:
                        irStrm = dev.infrared
                        cv2.namedWindow('IRStream')
                        cv2.imshow('IRStream', irStrm)

                    #wait and check if 'q' was pressed. If so, end streams
                    keyPress = cv2.waitKey(1) & 0xFF
                    if keyPress == ord('q'):
                        break

                    # if saving frames is requested, save desired streams
                    if saveRate and cnt % saveRate == 0:
                        if self.streamDepth:
                            dname = "./frames/depth/frame%d.jpg" % cnt
                            cv2.imwrite(dname, dep)
                            ret, dTest = cv2.threshold(dep, 2, 10, cv2.THRESH_TOZERO_INV)
                            cv2.imwrite(dname, dTest)
                            np.savetxt('./frames/depth/thresh_depvals%d.txt' % cnt, dTest)
                            np.savetxt('./frames/depth/unproc_depvals%d.txt' % cnt, dTest)

                        if self.streamColor:
                            cname = "./frames/color/frame%d.jpg" % cnt
                            gname = "./frames/gray/frame%d.jpg" % cnt
                            cv2.imwrite(cname, color)
                            cv2.imwrite(gname, grayscale)
