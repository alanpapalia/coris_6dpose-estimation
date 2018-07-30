#!/bin/bash

# bash script that runs realsense testing, makes
# video and puts video and frames in specific
# directories for each trial

baseDir="./frames/single_camera/"

colorInDir=$baseDir"color/"
grayInDir=$baseDir"/gray/"
depthInDir=$baseDir"depth/"
ptsInDir=$baseDir"points/"

vidName="output.mp4"
copyDir=$outDir"/frameData/"

echo "Enter the name of the trial number"
read trialLabel

colorOutDir=$baseDir"trials/"$trialLabel"/color/"
grayOutDir=$baseDir"trials/"$trialLabel"/gray/"
depthOutDir=$baseDir"trials/"$trialLabel"/depth/"
ptsOutDir=$baseDir"trials/"$trialLabel"/pts/"

mkdir $colorInDir 2>/dev/null
mkdir $grayInDir 2>/dev/null
mkdir $depthInDir 2>/dev/null
mkdir $ptsInDir 2>/dev/null
mkdir $baseDir"trials/" 2>/dev/null

# "Not enough arguments!!"
# "1) number of cameras (0/1/2)"
# "2) How many cameras are you using? (0/1/2)"
# "3) Do you want to stream color data? (y/n)"
# "4) Do you want to stream depth data? (y/n)"
# "5) Do you want to stream point data? (y/n)"
# "6) Frame save rate? (0 for no save)"
# "7) Camera Mode\n	(1) Stream Video\n   (2) Save Feed\n   (3) Stream Video and Save Feed"

# Arguments for script are seen above
# Currently built for only single camera use

python realsense_script.py 1 y y n 1 3

echo
echo "Converting Frames into Video"
echo

mkdir $baseDir"trials/"$trialLabel 2>/dev/null
mkdir $colorOutDir 2>/dev/null
mkdir $grayOutDir 2>/dev/null
mkdir $depthOutDir 2>/dev/null
mkdir $ptsOutDir 2>/dev/null

# for dynamically input frame rate
# sudo ffmpeg -framerate $fRate -i $inDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $outDir$vidName
# sudo ffmpeg -framerate 30 -i $colorInDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $baseDir$trialLabel"/"$vidName

# copy all of most recent trial into saved trial directories
cp $colorInDir*".jpg" $colorOutDir  2>/dev/null
cp $colorInDir*".png" $colorOutDir 2>/dev/null
cp $grayInDir*".jpg" $grayOutDir 2>/dev/null
cp $depthInDir*".txt" $depthOutDir 2>/dev/null
cp $depthInDir*".png" $depthOutDir 2>/dev/null
cp $ptsInDir*".txt" $ptsOutDir 2>/dev/null
cp $ptsInDir*".pts" $ptsOutDir 2>/dev/null

# uncomment if want to generate xyz pointcloud
# python pcl_testing.py $ptsOutDir

# uncomment if want to generate xyzrgb pointcloud
# python3 makeRGBXYZ.py $ptsOutDir"/xyzrgb3.txt"

# sudo rm -f "./frames/single_camera/"*
# sudo rm -f "./frames/two_camera/"*


