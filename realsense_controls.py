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
    color_stream = pyrs.stream.ColorStream()  # rg color image

    def __init__(self):
        self.strms = []

    # add depth stream to camera
    def addDepStream(self):
        self.strms.append(self.depth_stream)
        self.strms.append(self.dac_stream)

    # add depth stream to camera
    def addColorStream(self):
        self.strms.append(self.color_stream)

    # create camera instance w/ color and adjusted depth streams
    # Params: saveRate - how many frames between saving to file (0 for no save)
    def startColorAndDepthStreams(self, saveRate):
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

                new_image = [[0.0] * 640] * 480

                while True:

                    cnt += 1
                    if (cnt % 10) == 0:
                        now = time.time()
                        dt = now - last
                        fps = 10/dt
                        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
                        last = now

                    dev.wait_for_frames()
                    color = dev.color
                    dep = dev.dac


                    d = (dev.dac * dev.depth_scale)

                    f = open('testfile.txt', 'w')
                    for i in range(len(d)):
                        for j in range(len(d[0])):
                            new_image[i][j] = d[i][j]
                            f.write(str(i) + ', ' + str(j) + ': ')
                            if d[i][j] != 0:
                                # f.write('{:.5}'.format((d[i][j])))
                                f.write(str(d[i][j]))
                            else:f.write('0')
                            f.write('\n')
                        f.write('\n')
                    cv2.imwrite('testfile.bmp', (dev.dac))
                    # f = cv2.FileStorage('alanimg.yml', flags = 1)
                    # f.write(name = 'matrix', val = d)
                    # f.release()


                    grayscale = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
                    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

                    dep = convert_z16_to_bgr(dep)
                    dep = cv2.cvtColor(dep, cv2.COLOR_BGR2GRAY)

                    dep = cv2.adaptiveThreshold(dep, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)
                    grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

                    cv2.namedWindow('ColorStream')
                    cv2.imshow('ColorStream', color)
                    cv2.namedWindow('DepthStream')
                    cv2.imshow('DepthStream', dep)
                    cv2.namedWindow('GrayStream')
                    cv2.imshow('GrayStream', grayscale)

                    keyPress = cv2.waitKey(1) & 0xFF

                    if keyPress == ord('q'):
                        break

                    if saveRate and cnt%saveRate == 0:
                        dname = "./frames/depth/frame%d.jpg"%cnt
                        cname = "./frames/color/frame%d.jpg"%cnt
                        gname = "./frames/gray/frame%d.jpg" % cnt
                        cv2.imwrite(dname, dep)
                        cv2.imwrite(cname, color)
                        cv2.imwrite(gname, grayscale)

    # create camera instance w/ color depth streams
    # Params: saveRate - how many frames between saving to file (0 for no save)
    def startColorStream(self, saveRate):
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
                        fps = 10/dt
                        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
                        last = now

                    dev.wait_for_frames()
                    color = dev.color

                    grayscale = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
                    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

                    grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

                    cv2.namedWindow('ColorStream')
                    cv2.imshow('ColorStream', color)
                    cv2.namedWindow('GrayStream')
                    cv2.imshow('GrayStream', grayscale)

                    keyPress = cv2.waitKey(1) & 0xFF
                    if keyPress == ord('q'):
                        break

                    if saveRate and cnt%saveRate == 0:
                        cname = "./frames/color/frame%d.jpg"%cnt
                        gname = "./frames/gray/frame%d.jpg" % cnt
                        cv2.imwrite(cname, color)
                        cv2.imwrite(gname, grayscale)


    # create camera instance w/ color and adjusted depth streams
    # Params: saveRate - how many frames between saving to file (0 for no save)
    def startDepthStreams(self, saveRate):
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
                    dep = dev.dac
                    dep = cv2.cvtColor(dep, cv2.COLOR_BGR2GRAY)
                    dep = cv2.adaptiveThreshold(dep, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

                    cv2.namedWindow('DepthStream')
                    cv2.imshow('DepthStream', dep)

                    keyPress = cv2.waitKey(1) & 0xFF
                    if keyPress == ord('q'):
                        break

                    if saveRate and cnt % saveRate == 0:
                        dname = "./frames/depth/frame%d.jpg" % cnt
                        cv2.imwrite(dname, dep)
