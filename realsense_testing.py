import logging
logging.basicConfig(level=logging.INFO)

import os, shutil
import time
import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option
import hand_segmentation as hs

depth_stream = pyrs.stream.DepthStream() #depth image taken straight from ir cameras
dac_stream = pyrs.stream.DACStream()     #depth image corrected to pair w/ color_stream (fixes camera offset)
color_stream = pyrs.stream.ColorStream() #rg color image

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

def promptClearImages():
    print('Do you want to clear image directory? (y/n)')
    if raw_input() == 'y':
        depth_folder = './frames/depth/'
        color_folder = './frames/color/'
        gray_folder = './frames/gray/'

        dirs = [depth_folder, color_folder, gray_folder]
        for folder in dirs:
            for file in os.listdir(folder):
                file_path = os.path.join(folder, file)
                try:
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                except Exception as e:
                    print(e)












promptClearImages()
with pyrs.Service() as serv:
    with serv.Device(streams=(depth_stream, color_stream, dac_stream)) as dev:

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
            dep = dev.dac

            grayscale = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

            dep = convert_z16_to_bgr(dep)
            dep = cv2.cvtColor(dep, cv2.COLOR_BGR2GRAY)

            dep = cv2.adaptiveThreshold(dep, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)
            grayscale = cv2.adaptiveThreshold(grayscale, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

            # cv2.putText(cd, str(fps_smooth)[:4], (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0))
            # cd = cv2.cvtColor(cd, cv2.COLOR_BGR2GRAY)
            # print(cd.shape)

            cv2.namedWindow('ColorStream')
            cv2.imshow('ColorStream', color)
            cv2.namedWindow('DepthStream')
            cv2.imshow('DepthStream', dep)
            cv2.namedWindow('GrayStream')
            cv2.imshow('GrayStream', grayscale)

            keyPress = cv2.waitKey(1) & 0xFF

            if keyPress == ord('q'):
                break
            # elif keyPress == ord('c'):
            if cnt%30 == 0:
                dname = "./frames/depth/frame%d.jpg"%cnt
                cname = "./frames/color/frame%d.jpg"%cnt
                gname = "./frames/gray/frame%d.jpg" % cnt
                cv2.imwrite(dname, dep)
                cv2.imwrite(cname, color)
                cv2.imwrite(gname, grayscale)

    hs.clearImages()
    hs.depthImageSegmentation()
