import pcl
import os
import numpy as np
import sys

ptsFolder = './frames/single_camera/points/'

def generatePointClouds(ptsDir):
    pcd_folder = fileDir+'../PCD/'
    if not os.path.exists(pcd_folder):
        os.makedirs(pcd_folder)

    file_tracker = 0
    pcdArr = []
    for file_name in os.listdir(ptsFolder):
        if file_tracker == 0:
            pcdArr = np.loadtxt(ptsFolder+file_name, dtype=np.float32)
            file_tracker += 1
        else:
            image_points = np.loadtxt(ptsFolder+file_name, dtype=np.float32) 
            pcdArr = np.concatenate((pcdArr, image_points))        

    pclObject = pcl.PointCloud()
    pclObject.from_array(np.array(pcdArr, dtype=np.float32))
    pclObject.to_file(pcd_folder + 'PCDFile' + str(file_tracker) + '.pcd')
    pcl.save(pclObject, pcd_folder + 'PLYFile' + str(file_tracker) + '.ply')

# can receive file directory as arg
# defaults to ptsFolder if no dir specific
if len(sys.argv) == 2:
    fileDir = sys.argv[1]
    generatePointClouds(fileDir)    
else:
    generatePointClouds(ptsFolder)    