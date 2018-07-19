#!/bin/bash

python realsense_testing.py

echo
echo "Converting Frames into Video"
echo
echo "Enter the name of the trial number"
read trialLabel

inDir="./frames/single_camera/color/"
outDir=$inDir$trialLabel"/"
vidName="output.mp4"
copyDir=$outDir"/frameData/"

mkdir $outDir
mkdir $copyDir

sudo ffmpeg -framerate 5 -i $inDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $outDir$vidName
cp $inDir*".jpg" $copyDir
sudo rm -rf $inDir*".jpg"
