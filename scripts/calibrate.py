#!/usr/bin/env python

import sys
import os
import yaml
import numpy as np
import glob
import cv2
import pprint
import csv

import cv_bridge
import rosbag
import sensor_msgs
import sensor_msgs.msg

global Kr
global Kd
global p_world
global p_camera

def calibrate():

    global Kr
    global Kd
    global p_world
    global p_camera

    print 'here'

def load_images(f):

    # bag file 
    bag = rosbag.Bag(f, 'r')

    # cv bridge object
    bridge = cv_bridge.CvBridge()

    # extract rgb image
    msg = sensor_msgs.msg.Image()
    for topic, msg, t in bag.read_messages(topics='/camera/rgb/image_raw'):
        pass

    I_rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # extract depth image
    msg = sensor_msgs.msg.Image()
    for topic, msg, t in bag.read_messages(topics='/camera/depth/image_raw'):
        pass

    I_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    return I_rgb, I_depth

def load_ground_truth(f):

    gt = []
    with open(f, 'r') as fstream:

        # strip first line
        header = fstream.readline()

        # for each line
        for line in fstream:

            # clean the junk
            cleanline = line.replace('\n', '').replace(' ', '')

            # parse
            gt.append(cleanline.split(',')[1:-1])

    return np.array(gt)

def load_intrinsics(f):

    # parse
    calib = {}
    with open(f, 'r') as fstream:
        try:
            calib = yaml.load(fstream)
        except yaml.YAMLError as exception:
            print exception

    return calib

def mouse_callback(event, x, y, flags, param):

    global p_camera

    if event == cv2.EVENT_FLAG_LBUTTON:

        # append to list of points
        p_camera.append([x, y])

        # if we have enough points calibrate
        if (len(p_camera) == len(p_world)):
            calibrate()
            p_camera = []



if __name__ == '__main__':

    if len(sys.argv) < 5:
        print 'Usage: ' + os.path.basename(sys.argv[0]) + \
                ' <rgb_calibration.yaml>' + ' <depth_calibration.yaml>' + \
                ' <ground_truth.txt>' + ' <bag_file.bag>'
        sys.exit()

    global Kr
    global Kd
    global p_world
    global p_camera

    # intrinsic calibrations
    Kr = load_intrinsics(sys.argv[1])
    Kd = load_intrinsics(sys.argv[2])

    # load ground truth
    p_world = load_ground_truth(sys.argv[3])

    # load the images from the bag file
    [I_rgb, I_depth] = load_images(sys.argv[4])

    # init camera points
    p_camera = []

    # click and do stuff
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
    cv2.imshow('image', I_rgb)
    cv2.waitKey(0)



