import pcl
import os
import numpy as np

ptsFolder = './frames/single_camera/points/'
pcd_folder = './frames/single_camera/PCD/'

def generatePointClouds():
    file_tracker = 1
    for file_name in os.listdir(ptsFolder):
        image_points = np.loadtxt(ptsFolder+file_name, dtype=np.float32)
        pclObject = pcl.PointCloud()
        pclObject.from_array(np.array(image_points, dtype=np.float32))
        pclObject.to_file(pcd_folder + 'PCDFile' + str(file_tracker) + '.pcd')
        pcl.save(pclObject, pcd_folder + 'PLYFile' + str(file_tracker) + '.ply')
        file_tracker += 1


generatePointClouds()    