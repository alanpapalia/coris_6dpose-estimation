#!/bin/bash

# bash script that runs realsense testing, makes
# video and puts video and frames in specific
# directories for each trial



echo "Enter the name of the trial"
read trialLabel

nCams=2

streamColor='y'
streamDepth='y'
streamPts='n'

# how often to save frames 
# (1 -> every frame, 2 -> every other frame, etc
# if == 0 -> no frames)
# Recordings are taken at 30 fps
# so saveFrames=5 -> 30/5 = 6 fps saved
saveFrames=5

# command to protect script from quitting by CTRL+C/Z uncomment at will
# trap "echo CTRL+C disabled. Please exit by pressing 'q'" INT TSTP

if [[ nCams -eq 1 ]]; then
	baseDir="./frames/single_camera/"
	mkdir $baseDir 2>/dev/null

	python realsense_script.py $nCams $streamColor $streamDepth $streamPts $saveFrames $trialLabel

	vidName="output.mp4"
	# makes movie from color images, assumes 30 fps
	# sudo ffmpeg -framerate 30 -i $colorInDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $baseDir$trialLabel"/"$vidName
fi
if [[ nCams -eq 2 ]]; then
	baseDir="./frames/two_camera/"
	mkdir $baseDir 2>/dev/null

	python realsense_script.py $nCams $streamColor $streamDepth $streamPts $saveFrames $trialLabel

	vidName="output.mp4"
	# makes movie from color images, assumes 30 fps
	# sudo ffmpeg -framerate 30 -i $colorInDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $baseDir$trialLabel"/"$vidName
fi

