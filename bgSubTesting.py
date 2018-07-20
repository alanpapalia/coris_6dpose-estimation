import image_controls as imControl

baseDir = './frames/single_camera/color/'
tDir = raw_input("Which trial would you like to test? (t1/t2/t3/t4)")
frameDir = baseDir+tDir


frames = imControl.getFrameList(frameDir)
imControl.runBGSubMOG2(frames)

return