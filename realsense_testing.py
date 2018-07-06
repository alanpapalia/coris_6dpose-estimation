import logging
import realsense_controls as rsc
import image_controls as imControl
import tracking

logging.basicConfig(level=logging.INFO)

tracking_dir = './frames/color/'

# prompt to determine stream data to save
print("Do you want to stream color data? (y/n)")
colStrm = raw_input() == 'y'
print("Do you want to stream depth data? (y/n)")
depStrm = raw_input() == 'y'
print("Frame save rate? (0 for no save)")
saveRate = raw_input()
try: saveRate = int(saveRate)
except: saveRate = 0

fps = 0.25

# if want to save frames, first clear out old ones
if saveRate != 0:
    imControl.clearTestImages()
    fps = saveRate/30

# initialize realsense device
rs = rsc.RSControl()


if colStrm and depStrm:
    rs.addColorStream()
    rs.addDepStream()
    rs.startStreams(saveRate)
elif colStrm:
    rs.addColorStream()
    rs.startStreams(saveRate)
elif depStrm:
    rs.addDepStream()
    rs.startStreams(saveRate)


# pcd.generatePointClouds('./frames/depth')
print("Do you want to segment the depth images? (y/n)")
if raw_input() == 'y':
    imControl.clearImages()  # erases previously segmented images
    imControl.colBasedDepImgSeg()



# print("Do you want to run tracking? (y/n)")
# if raw_input() == 'y':
    # print("How many frames to remove from the start?")
    # nRem = int(raw_input())
    # tracking.clearUntrackedImg(tracking_dir, nRem)
    # tracking.trackColorImg(tracking_dir, fps)
