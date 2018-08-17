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

# camera streaming mode
# (1 -> stream only, 2 -> save images only, 3 -> stream and save imgs)
streamMode=3	

trap "echo CTRL+C disabled. Please exit by pressing 'q'" INT TSTP

if [[ nCams -eq 1 ]]; then
	baseDir="./frames/single_camera/"

	colorInDir=$baseDir"color/"
	grayInDir=$baseDir"/gray/"
	depthInDir=$baseDir"depth/"
	ptsInDir=$baseDir"points/"

	rm -rf	$colorInDir*"/"*".jpg" $colorInDir*"/"*".txt" $colorInDir*"/"*".png"
	rm -rf  $grayInDir*"/"*".jpg" $grayInDir*"/"*".txt" $grayInDir*"/"*".png"
	rm -rf  $depthInDir*"/"*".jpg" $depthInDir*"/"*".txt" $depthInDir*"/"*".png"
	rm -rf  $ptsInDir*"/"*".jpg" $ptsInDir*"/"*".txt" $ptsInDir*"/"*".png"

	mkdir $colorInDir 2>/dev/null
	mkdir $grayInDir 2>/dev/null
	mkdir $depthInDir 2>/dev/null
	mkdir $ptsInDir 2>/dev/null
	mkdir $baseDir"trials/" 2>/dev/null

	vidName="output.mp4"
	copyDir=$outDir"/frameData/"
	colorOutDir=$baseDir"trials/"$trialLabel"/color/"
	grayOutDir=$baseDir"trials/"$trialLabel"/gray/"
	depthOutDir=$baseDir"trials/"$trialLabel"/depth/"
	ptsOutDir=$baseDir"trials/"$trialLabel"/pts/"

	mkdir $baseDir"trials/"$trialLabel 2>/dev/null
	mkdir $colorOutDir 2>/dev/null
	mkdir $grayOutDir 2>/dev/null
	mkdir $depthOutDir 2>/dev/null
	mkdir $ptsOutDir 2>/dev/null

	python realsense_script.py $nCams $streamColor $streamDepth $streamPts $saveFrames $streamMode 

	# makes movie from color images, assumes 30 fps
	# sudo ffmpeg -framerate 30 -i $colorInDir"frame%00d.jpg" -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p $baseDir$trialLabel"/"$vidName

	# copy all of most recent trial into saved trial directories
	cp $colorInDir*".jpg" $colorOutDir  2>/dev/null
	cp $colorInDir*".png" $colorOutDir 2>/dev/null
	cp $grayInDir*".jpg" $grayOutDir 2>/dev/null
	cp $depthInDir*".txt" $depthOutDir 2>/dev/null
	cp $depthInDir*".png" $depthOutDir 2>/dev/null
	cp $ptsInDir*".txt" $ptsOutDir 2>/dev/null
	cp $ptsInDir*".pts" $ptsOutDir 2>/dev/null
	cp $ptsInDir*".ply" $ptsOutDir 2>/dev/null
fi
if [[ nCams -eq 2 ]]; then
	baseDir="./frames/two_camera/"
	mkdir $baseDir 2>/dev/null


	colorInDir=$baseDir"color"
	grayInDir=$baseDir"gray"
	depthInDir=$baseDir"depth"
	ptsInDir=$baseDir"points"
	timeInDir=$baseDir"time"

	rm -rf	$colorInDir*"/"*".jpg" $colorInDir*"/"*".txt" $colorInDir*"/"*".png"
	rm -rf  $grayInDir*"/"*".jpg" $grayInDir*"/"*".txt" $grayInDir*"/"*".png"
	rm -rf  $depthInDir*"/"*".jpg" $depthInDir*"/"*".txt" $depthInDir*"/"*".png"
	rm -rf  $ptsInDir*"/"*".ply" $ptsInDir*"/"*".txt" $ptsInDir*"/"*".png"
	rm -rf  $timeInDir*".txt"

	mkdir $colorInDir"1" 2>/dev/null
	mkdir $grayInDir"1" 2>/dev/null
	mkdir $depthInDir"1" 2>/dev/null
	mkdir $ptsInDir"1" 2>/dev/null
	mkdir $colorInDir"2" 2>/dev/null
	mkdir $grayInDir"2" 2>/dev/null
	mkdir $depthInDir"2" 2>/dev/null
	mkdir $ptsInDir"2" 2>/dev/null
	mkdir $timeInDir 2>/dev/null
	mkdir $baseDir"trials/" 2>/dev/null

	vidName="output.mp4"
	copyDir=$outDir"/frameData/"
	colorOutDir=$baseDir"trials/"$trialLabel"/color"
	grayOutDir=$baseDir"trials/"$trialLabel"/gray"
	depthOutDir=$baseDir"trials/"$trialLabel"/depth"
	ptsOutDir=$baseDir"trials/"$trialLabel"/points"
	timeOutDir=$baseDir"trials/"$trialLabel"/time"

	mkdir $baseDir"trials/"$trialLabel 2>/dev/null
	mkdir $colorOutDir"1" 2>/dev/null
	mkdir $grayOutDir"1" 2>/dev/null
	mkdir $depthOutDir"1" 2>/dev/null
	mkdir $ptsOutDir"1" 2>/dev/null
	mkdir $colorOutDir"2" 2>/dev/null
	mkdir $grayOutDir"2" 2>/dev/null
	mkdir $depthOutDir"2" 2>/dev/null
	mkdir $ptsOutDir"2" 2>/dev/null
	mkdir $timeOutDir 2>/dev/null

	python realsense_script.py $nCams $streamColor $streamDepth $streamPts $saveFrames $streamMode 

	cp $colorInDir"1/"*".jpg" $colorOutDir"1"  2>/dev/null
	cp $colorInDir"1/"*".png" $colorOutDir"1" 2>/dev/null
	cp $grayInDir"1/"*".jpg" $grayOutDir"1" 2>/dev/null
	cp $depthInDir"1/"*".txt" $depthOutDir"1" 2>/dev/null
	cp $depthInDir"1/"*".png" $depthOutDir"1" 2>/dev/null
	cp $ptsInDir"1/"*".txt" $ptsOutDir"1" 2>/dev/null
	cp $ptsInDir"1/"*".pts" $ptsOutDir"1" 2>/dev/null
	cp $ptsInDir"1/"*".ply" $ptsOutDir"1" 2>/dev/null
	cp $colorInDir"2/"*".jpg" $colorOutDir"2"  2>/dev/null
	cp $colorInDir"2/"*".png" $colorOutDir"2" 2>/dev/null
	cp $grayInDir"2/"*".jpg" $grayOutDir"2" 2>/dev/null
	cp $depthInDir"2/"*".txt" $depthOutDir"2" 2>/dev/null
	cp $depthInDir"2/"*".png" $depthOutDir"2" 2>/dev/null
	cp $ptsInDir"2/"*".txt" $ptsOutDir"2" 2>/dev/null
	cp $ptsInDir"2/"*".pts" $ptsOutDir"2" 2>/dev/null
	cp $ptsInDir"2/"*".ply" $ptsOutDir"2" 2>/dev/null
	cp $timeInDir"/"*".txt" $timeOutDir 2>/dev/null

fi

