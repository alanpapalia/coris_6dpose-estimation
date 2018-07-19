#!/bin/bash

python realsense_testing.py

echo
echo "Converting Frames into Video"
echo
echo "Enter the name of the trial number"
read trialLabel

# echo "Enter the captured frame rate (fps)"
# read fRate


inDir="./frames/single_camera/color/"
outDir=$inDir$trialLabel"/"
vidName="output.mp4"
copyDir=$outDir"/frameData/"

mkdir $outDir
mkdir $copyDir

# top for dynamically input frame rate
# sudo ffmpeg -framerate $fRate -i $inDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $outDir$vidName
sudo ffmpeg -framerate 30 -i $inDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $outDir$vidName

cp $inDir*".jpg" $copyDir
sudo rm -rf $inDir*".jpg"
clear