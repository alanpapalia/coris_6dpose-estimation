import os, cv2, numpy
import rospy
import roslib

import std_msgs.msg

fx = 581.6880493164062
fy = 581.6880493164062
ppx = 317.5009460449219
ppy = 239.8851318359375


def generatePointClouds(folder):

    pcd_folder = folder + '/pcd_files/'
    cnt = 0

    for file_name in os.listdir(folder):
        cnt+=1
        imageDepth = cv2.imread(folder + file_name, 2)
        image_points = [[0,0,0]] * (640 * 480)
        tracker = 0

        for i in range(len(imageDepth)):
            for j in range(len(imageDepth[0])):
                depth_value = float(imageDepth[i][j])
                X = float((i - ppx)) * float(depth_value) / float(fx)
                Y = float((j - ppy)) * float(depth_value) / float(fy)
                image_points[tracker] = [X, Y, depth_value]
                tracker += 1



        pclObject = pcl.PointCloud()
        pclObject.from_array(numpy.array(image_points, dtype=numpy.float32))
        #fil = pclObject.make_statistical_outlier_filter()
        #fil.set_mean_k (50)
        #fil.set_std_dev_mul_thresh (3.0)
        #filteredPclObject = fil.filter()
        pclObject.to_file(pcd_folder + 'pcd_img' + str(cnt) + '.pcd')
        pcl.save(pclObject, pcd_folder + 'PLYFile' + str(cnt) + '.ply')
