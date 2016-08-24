#!/usr/bin/env python

import sys
import os
import yaml
import numpy as np
import glob
import cv2
import pprint

import cv_bridge
import rosbag
import sensor_msgs
import sensor_msgs.msg

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

    if event == cv2.EVENT_FLAG_LBUTTON:
        print x, y


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print 'Usage: ' + os.path.basename(sys.argv[0]) + ' <directory>'
        sys.exit()

    # get absolute path
    abspath = os.getcwd() + '/' + sys.argv[1]

    # intrinsic calibrations
    C = {}

    # for each yaml file in the directory
    for f in glob.glob(abspath + '*.yaml'):

        # intrinsics type
        t = f.split('/')[-1].split('_')[0]

        C[t] = load_intrinsics(f)

    # pprint.pprint(C)

    # load the images from the bag file
    f = glob.glob(abspath + '*.bag')[0]
    [I_rgb, I_depth] = load_images(f)

    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)

    cv2.imshow('image', I_rgb)
    cv2.waitKey(0)



