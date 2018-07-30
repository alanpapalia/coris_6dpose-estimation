"""
This is the main python script which should be used manually
for camera testing. It calls all auxilliary scripts to perform
object tracking through the Intel RealSense ZR300 Cameras
"""

import logging
import realsense_controls as rsc
import image_controls as imControl
import tracking



logging.basicConfig(level=logging.INFO)

color_dir = './frames/color/'
depth_dir = './frames/depth/'

# initialize realsense device
rs = rsc.RSControl()

nCams = int(raw_input("How many cameras are you using? (0/1/2)"))

# prompt to determine stream data to save
print("Do you want to stream color data? (y/n)")
if raw_input() == 'y':
    rs.addColorStream()

print("Do you want to stream depth data? (y/n)")
if raw_input() == 'y':
    rs.addDepStream()

print("Do you want to stream point data? (y/n)")
if raw_input() == 'y':
    rs.addPointStream()

print("Frame save rate? (0 for no save)")
saveRate = raw_input()
try:
    saveRate = int(saveRate)
except:
    saveRate = 0

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

# nCams = 1
# rs.addColorStream()
# rs.addDepStream()
# rs.addPointStream()
# saveRate = 1;

# if want to save frames, first clear out old ones
if saveRate != 0:
    imControl.clearTestImages(nCams)

if nCams > 0:
	# rs.startStreams(saveRate, nCams)
	rs.saveFeed(saveRate, nCams)
	# rs.startStreamAndSave(saveRate, nCams)

# if segDepImg == 'y':
#     imControl.clearImages()  # erases previously segmented images
#     imControl.colBasedDepImgSeg()

# pcd.generatePointClouds('./frames/depth')
