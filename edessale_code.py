#!/usr/bin/env python
import argparse
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import    Image
import time
import os
import math
import tf
import openravepy
from openravepy import *
import signal
import sys
import ctypes
from ctypes import *
from geometry_msgs.msg import (Point, PointStamped, PoseStamped)
from std_msgs.msg import Header
import pyrealsense as pyrs
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('BAXTER')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
rs.enable()
pyrs.start()
dev = pyrs.Device()
trackRGB = trackDepth = -2
index = -1
suffix = ''
arm = ''
a = time.time()
os.mkdir('/home/edessale/ros_ws/src/vision_matlab/training/%f/' % a)
env = openravepy.Environment()
env.StopSimulation()
env.Load('/home/edessale/ros_ws/src/openrave/python/baxterXMLs/Arms.xml')

rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, rgb_save)
depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, depth_save)

def rgb_save():
    global dev, trackRGB, suffix, arm, a
    dev.wait_for_frame()
    c = dev.colour
     c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
    cv2.imwrite('/home/edessale/ros_ws/src/vision_matlab/training/%f/%s_%s_rgb_%d.png' % (a, suffix, arm, trackRGB), c)
    trackRGB += 1

def depth_save():
    global dev, trackDepth, suffix, arm, a
    dev.wait_for_frame()
        #print type(dev.depth)
        #print dev.depth_scale
        d = dev.dac * dev.depth_scale * 1000
        #cv.Mat depth_file(rows, cols, CV_32FC1) #CV_32FC1
        blank_image = np.zeros((480,640), np.uint8)
        for i in range(len(d)):
            for j in range(len(d[0])):
                blank_image[i][j] = d[i][j]

    cv2.imwrite('/home/edessale/ros_ws/src/vision_matlab/training/%f/%s_%s_depth_%d.png' % (a, suffix, arm, trackRGB), dev.dac)
    trackDepth += 1

def set_joint_positions(joints, arm, names, arm_robot):
    names[arm + '_s0'] =joints[0]
    names[arm + '_s1'] =joints[1]
    names[arm + '_e0'] =joints[2]
    names[arm + '_e1'] =joints[3]
    names[arm + '_w0'] =joints[4]
    names[arm + '_w1'] =joints[5]
    names[arm + '_w2'] =joints[6]
    arm_robot.move_to_joint_positions(names, threshold = 0.02)
    rospy.sleep(1)

def getTransform(env, joints, arm):
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load('/home/edessale/ros_ws/src/openrave/python/baxterXMLs/Arms.xml')
    robot = env.GetRobots()[0]
    manip = robot.SetActiveManipulator(arm + '_arm')
    robot = env.GetRobots()[0]
    robot.SetDOFValues(joints, manip.GetArmIndices())
    print manip.GetEndEffectorTransform()


def collect_data_arm(env, start, end, arm):
    global trackDepth, trackRGB, index, suffix, arm

    limb_arm = baxter_interface.Limb(arm)
    joint_angles = limb_arm.joint_angles()
    robot = env.GetRobots()[0]
    manip = robot.SetActiveManipulator(arm + '_arm')
    index = start - 1
    trackRGB = trackDepth = index + 1

    tros2 = tf.TransformListener()
    tros2.setUsingDedicatedThread(True)
    tros2.waitForTransform('/base', '/' + arm + '_arm_mount', rospy.Time(), rospy.Duration(2))

    output = ctypes.CDLL('/home/edessale/ros_ws/src/openrave/python/' + arm + '.so')
    output.compute.restype = POINTER(c_float)
    output.compute.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]

    index = start - 1
    trackRGB = trackDepth = index + 1

    base_angle = tf.transformations.quaternion_from_euler(-1.3146304661619506, -1.430541711400685, -1.7206422170342097)
    pitch = tf.transformations.quaternion_from_euler(0,0.3,0)
    roll = tf.transformations.quaternion_from_euler(3.14,0,0)

    angles = [tf.transformations.quaternion_from_euler(-0.53178468585711, -0.6716203522516357, -2.26003918089504625), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.45)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.6)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0., -0.75)), tf.transformations.quaternion_from_euler(-0.53178468585711, -0.6716203522516357, -2.3003918089504625), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.45)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.6)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0., -0.75))] #change these viewpoint angles

    if arm == 'left':
        positions = [[0.71, 0.42,0.65112867],[0.66895583,0.361326311,0.32112867],[0.76895583,0.51326311,0.32112867],[0.7895583,0.551326311,0.32112867],[0.75,0.4,0.42112867],[0.66895583,0.41326311,0.12112867],[0.76895583,0.51326311,0.12112867],[0.7895583,0.551326311,0.12112867]]

    else:
        positions = [[0.75,-0.4,0.62112867],[0.66895583,-0.361326311,0.32112867],[0.76895583,-0.51326311,0.32112867],[0.7895583,-0.551326311,0.32112867],[0.75,-0.4,0.42112867],[0.66895583,-0.41326311,0.12112867],[0.76895583,-0.51326311,0.12112867],[0.7895583,-0.551326311,0.12112867]]


    for i in range(start, end):
        ind = 0
        poseStamped = PoseStamped()
        hdr2 = Header(stamp=rospy.Time(), frame_id='/base')
        poseStamped.header = hdr2
        poseStamped.pose.position.x = positions[i][0]
        poseStamped.pose.position.y = positions[i][1]
        poseStamped.pose.position.z = positions[i][2]
        poseStamped.pose.orientation.w = angles[i][0]
        poseStamped.pose.orientation.x = angles[i][1]
        poseStamped.pose.orientation.y = angles[i][2]
        poseStamped.pose.orientation.z = angles[i][3]
        result2 = tros2.transformPose('/' + arm + '_arm_mount', poseStamped)
        res = output.compute(result2.pose.position.x, result2.pose.position.y, result2.pose.position.z, result2.pose.orientation.w, result2.pose.orientation.x, result2.pose.orientation.y, result2.pose.orientation.z)
        k = int(res[999]/7)-10
        new_array = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))

        for z in range(0, len(new_array)):
            robot.SetDOFValues(new_array[z], manip.GetArmIndices())
            if checkJoints(joints[z]) and not(robot.CheckSelfCollision()):
                joints = joints[z]

        if arm == 'left':
            joints[0] *= -1
            joints[2] *= -1
            joints[4] *= -1
            joints[6] *= -1

        print joints
        set_joint_positions(joints, arm, joint_angles, limb_arm)
        print i

        suffix = getSuffix(i)

        trackRGB = trackDepth = i
        index += 1

        depth_save()
        rgb_save()
        while(trackRGB == index or trackDepth == index):
            a = 1


def getSuffix(i):
    if i >= 0 and i <= 3:
        suffix = 'bin1'

    if i >= 4 and i <= 7:
        suffix = 'bin2'

    if i >= 8 and i <= 11:
        suffix = 'bin3'

    if i >= 12 and i <= 15:
        suffix = 'bin4'

    if i >= 16 and i <= 19:
        suffix = 'bin5'

    if i >= 20 and i <= 23:
        suffix = 'bin6'

    return suffix

def checkJoints(angles):
    if -1.7 <= angles[0] <= 1.7 and -2.147 <= angles[1] <= 1.047 and -3.028 <= angles[2] <= 3.028 and -0.05 <= angles[3] <= 2.618 and -3.059 <= angles[4] <= 3.059 and -1.571 <= angles[5] <= 2.094 and -3.059 <= angles[6] <= 3.059:
        return True
    return False

def exit_gracefully(signum, frame):#!/usr/bin/env python
import argparse
import numpy as np
import rospy
from roslib import message
import cv2
from cv_bridge import CvBridge, CvBridgeError
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_external_devices
from sensor_msgs.msg import    Image
import time
import os
import math
import tf
import openravepy
from openravepy import *
import signal
import sys
import ctypes
from ctypes import *
from geometry_msgs.msg import (Point, PointStamped, PoseStamped)
from std_msgs.msg import Header
import pyrealsense as pyrs
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('BAXTER')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
rs.enable()
pyrs.start()
dev = pyrs.Device()
trackRGB = trackDepth = -2
index = -1
suffix = ''
arm = ''
a = time.time()
os.mkdir('/home/edessale/ros_ws/src/vision_matlab/training/%f/' % a)
env = openravepy.Environment()
env.StopSimulation()
env.Load('/home/edessale/ros_ws/src/openrave/python/baxterXMLs/Arms.xml')

rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, rgb_save)
depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, depth_save)

def rgb_save():
    global dev, trackRGB, suffix, arm, a
    dev.wait_for_frame()
        c = dev.colour
     c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
    cv2.imwrite('/home/edessale/ros_ws/src/vision_matlab/training/%f/%s_%s_rgb_%d.png' % (a, suffix, arm, trackRGB), c)
    trackRGB += 1

def depth_save():
    global dev, trackDepth, suffix, arm, a
    dev.wait_for_frame()
        #print type(dev.depth)
        #print dev.depth_scale
        d = dev.dac * dev.depth_scale * 1000
        #cv.Mat depth_file(rows, cols, CV_32FC1) #CV_32FC1
        blank_image = np.zeros((480,640), np.uint8)
        for i in range(len(d)):
            for j in range(len(d[0])):
                blank_image[i][j] = d[i][j]

    cv2.imwrite('/home/edessale/ros_ws/src/vision_matlab/training/%f/%s_%s_depth_%d.png' % (a, suffix, arm, trackRGB), dev.dac)
    trackDepth += 1

def set_joint_positions(joints, arm, names, arm_robot):
    names[arm + '_s0'] =joints[0]
    names[arm + '_s1'] =joints[1]
    names[arm + '_e0'] =joints[2]
    names[arm + '_e1'] =joints[3]
    names[arm + '_w0'] =joints[4]
    names[arm + '_w1'] =joints[5]
    names[arm + '_w2'] =joints[6]
    arm_robot.move_to_joint_positions(names, threshold = 0.02)
    rospy.sleep(1)

def getTransform(env, joints, arm):
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load('/home/edessale/ros_ws/src/openrave/python/baxterXMLs/Arms.xml')
    robot = env.GetRobots()[0]
    manip = robot.SetActiveManipulator(arm + '_arm')
    robot = env.GetRobots()[0]
    robot.SetDOFValues(joints, manip.GetArmIndices())
    print manip.GetEndEffectorTransform()


def collect_data_arm(env, start, end, arm):
    global trackDepth, trackRGB, index, suffix, arm

    limb_arm = baxter_interface.Limb(arm)
    joint_angles = limb_arm.joint_angles()
    robot = env.GetRobots()[0]
    manip = robot.SetActiveManipulator(arm + '_arm')
    index = start - 1
    trackRGB = trackDepth = index + 1

    tros2 = tf.TransformListener()
    tros2.setUsingDedicatedThread(True)
    tros2.waitForTransform('/base', '/' + arm + '_arm_mount', rospy.Time(), rospy.Duration(2))

    output = ctypes.CDLL('/home/edessale/ros_ws/src/openrave/python/' + arm + '.so')
    output.compute.restype = POINTER(c_float)
    output.compute.argtypes = [c_float, c_float, c_float, c_float, c_float, c_float, c_float]

    index = start - 1
    trackRGB = trackDepth = index + 1

    base_angle = tf.transformations.quaternion_from_euler(-1.3146304661619506, -1.430541711400685, -1.7206422170342097)
    pitch = tf.transformations.quaternion_from_euler(0,0.3,0)
    roll = tf.transformations.quaternion_from_euler(3.14,0,0)

    angles = [tf.transformations.quaternion_from_euler(-0.53178468585711, -0.6716203522516357, -2.26003918089504625), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.45)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.6)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0., -0.75)), tf.transformations.quaternion_from_euler(-0.53178468585711, -0.6716203522516357, -2.3003918089504625), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.45)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0, -0.6)), quatMultiply(base_angle, tf.transformations.quaternion_from_euler(3.14,0., -0.75))] #change these viewpoint angles

    if arm == 'left':
        positions = [[0.71, 0.42,0.65112867],[0.66895583,0.361326311,0.32112867],[0.76895583,0.51326311,0.32112867],[0.7895583,0.551326311,0.32112867],[0.75,0.4,0.42112867],[0.66895583,0.41326311,0.12112867],[0.76895583,0.51326311,0.12112867],[0.7895583,0.551326311,0.12112867]]

    else:
        positions = [[0.75,-0.4,0.62112867],[0.66895583,-0.361326311,0.32112867],[0.76895583,-0.51326311,0.32112867],[0.7895583,-0.551326311,0.32112867],[0.75,-0.4,0.42112867],[0.66895583,-0.41326311,0.12112867],[0.76895583,-0.51326311,0.12112867],[0.7895583,-0.551326311,0.12112867]]


    for i in range(start, end):
        ind = 0
        poseStamped = PoseStamped()
        hdr2 = Header(stamp=rospy.Time(), frame_id='/base')
        poseStamped.header = hdr2
        poseStamped.pose.position.x = positions[i][0]
        poseStamped.pose.position.y = positions[i][1]
        poseStamped.pose.position.z = positions[i][2]
        poseStamped.pose.orientation.w = angles[i][0]
        poseStamped.pose.orientation.x = angles[i][1]
        poseStamped.pose.orientation.y = angles[i][2]
        poseStamped.pose.orientation.z = angles[i][3]
        result2 = tros2.transformPose('/' + arm + '_arm_mount', poseStamped)
        res = output.compute(result2.pose.position.x, result2.pose.position.y, result2.pose.position.z, result2.pose.orientation.w, result2.pose.orientation.x, result2.pose.orientation.y, result2.pose.orientation.z)
        k = int(res[999]/7)-10
        new_array = np.ctypeslib.as_array(res, shape=(int(res[999]/7),7))

        for z in range(0, len(new_array)):
            robot.SetDOFValues(new_array[z], manip.GetArmIndices())
            if checkJoints(joints[z]) and not(robot.CheckSelfCollision()):
                joints = joints[z]

        if arm == 'left':
            joints[0] *= -1
            joints[2] *= -1
            joints[4] *= -1
            joints[6] *= -1

        print joints
        set_joint_positions(joints, arm, joint_angles, limb_arm)
        print i

        suffix = getSuffix(i)

        trackRGB = trackDepth = i
        index += 1

        depth_save()
        rgb_save()
        while(trackRGB == index or trackDepth == index):
            a = 1


def getSuffix(i):
    if i >= 0 and i <= 3:
        suffix = 'bin1'

    if i >= 4 and i <= 7:
        suffix = 'bin2'

    if i >= 8 and i <= 11:
        suffix = 'bin3'

    if i >= 12 and i <= 15:
        suffix = 'bin4'

    if i >= 16 and i <= 19:
        suffix = 'bin5'

    if i >= 20 and i <= 23:
        suffix = 'bin6'

    return suffix

def checkJoints(angles):
    if -1.7 <= angles[0] <= 1.7 and -2.147 <= angles[1] <= 1.047 and -3.028 <= angles[2] <= 3.028 and -0.05 <= angles[3] <= 2.618 and -3.059 <= angles[4] <= 3.059 and -1.571 <= angles[5] <= 2.094 and -3.059 <= angles[6] <= 3.059:
        return True
    return False

def exit_gracefully(signum, frame):
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    signal.signal(signal.SIGINT, exit_gracefully)


original_sigint = signal.getsignal(signal.SIGINT)
signal.signal(signal.SIGINT, exit_gracefully)


arm = raw_input("Enter an arm to work with:\n")
if arm == 'right' or arm == 'left':
    start_time = time.time()
    collect_data_arm(env, 0, 24, arm)

    time_elapsed = time.time() - start_time
    print time_elapsed

rgb_sub.unregister()
depth_sub.unregister()

    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    signal.signal(signal.SIGINT, exit_gracefully)


original_sigint = signal.getsignal(signal.SIGINT)
signal.signal(signal.SIGINT, exit_gracefully)


arm = raw_input("Enter an arm to work with:\n")
if arm == 'right' or arm == 'left':
    start_time = time.time()
    collect_data_arm(env, 0, 24, arm)

    time_elapsed = time.time() - start_time
    print time_elapsed

rgb_sub.unregister()
depth_sub.unregister()
