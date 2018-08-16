"""
This is the python script which can be called by
terminal or bash commands for the purpose of camera testing.
It calls all auxilliary scripts to perform
data collection through Intel RealSense ZR300 Cameras
"""

import logging
import realsense_controls as rsc
import image_controls as imControl
import tracking
import sys

if len(sys.argv) != 7:
	print "\n\nNot enough arguments!!"
	print "1) number of cameras (0/1/2)"
	print "2) How many cameras are you using? (0/1/2)"
	print "3) Do you want to stream color data? (y/n)"
	print "4) Do you want to stream depth data? (y/n)"
	print "5) Do you want to stream point data? (y/n)"
	print "6) Frame save rate? (0 for no save)"
	print "7) Camera Modes\n   (1) Stream Video\n   (2) Save Feed\n   (3) Stream Video and Save Feed"
	exit()




logging.basicConfig(level=logging.INFO)


# initialize realsense device
rs = rsc.RSControl()

nCams = int(sys.argv[1])

if sys.argv[2] == 'y':
    rs.addColorStream()

if sys.argv[3] == 'y':
    rs.addDepStream()

if sys.argv[4] == 'y':
    rs.addPointStream()

saveRate = sys.argv[5]
try:
    saveRate = int(saveRate)
except:
    saveRate = 0

# uncomment if wanting to make xyzrgb data
# requires color and pt streams
# currently not working
rs.setXYZRGB()

# print("Do you want to find the corners of the checkerboard? (y/n)")
# findCorners = raw_input()
# if findCorners == 'y':
#     rs.findCorners = True

# print("Do you want to segment the depth images? (y/n)")
# segDepImg = raw_input()
# if segDepImg == 'y':
#     rs.colBasDepSeg = True

# print("Do you want to run watershed segmentation of the color images? (y/n)")
# waterSeg = raw_input()
# if waterSeg == 'y':
#     rs.waterSeg = True


# print("Do you want to run color tracking? (y/n)")
# if raw_input() == 'y':
#     print("How many frames to remove from the start?")
#     nRem = int(raw_input())
#     tracking.runColorTracking(color_dir, nRem)

# print("Do you want to run depth tracking? (y/n)")
# if raw_input() == 'y':
#     print("How many frames to remove from the start?")
#     nRem = int(raw_input())
#     tracking.runDepthTracking(depth_dir, nRem)


# if want to save frames, first clear out old ones
if saveRate != 0:
    imControl.clearTestImages(nCams)

camMode = sys.argv[6]

try:
	camMode = int(camMode)
except Exception as e:
	print e
	exit()

if camMode == 1:
	rs.startStreams(saveRate, nCams)
elif camMode == 2:
	rs.saveFeed(saveRate, nCams)
else:
	rs.startStreamAndSave(saveRate, nCams)

# imControl.makeGrayFrames("./frames/single_camera/color/")
