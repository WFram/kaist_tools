# !/usr/bin/python
#
# Convert stereo camera images to a rosbag
#
# Running:
#
#   python3 stereo_cam_to_rosbag.py path/to/left/image/folder path/to/right/image/folder stereo_cam.bag
#

import rosbag, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys
import numpy as np

from time import sleep
from glob import glob
from os import path

import cv2
import re

class Undistort(object):

    def __init__(self, fin, scale=1.0, fmask=None):
        self.fin = fin
        # read in distort
        with open(fin, 'r') as f:
            #chunks = f.readline().rstrip().split(' ')
            header = f.readline().rstrip()
            chunks = re.sub(r'[^0-9,]', '', header).split(',')
            self.mapu = np.zeros((int(chunks[1]),int(chunks[0])),
                    dtype=np.float32)
            self.mapv = np.zeros((int(chunks[1]),int(chunks[0])),
                    dtype=np.float32)
            for line in f.readlines():
                chunks = line.rstrip().split(' ')
                self.mapu[int(chunks[0]),int(chunks[1])] = float(chunks[3])
                self.mapv[int(chunks[0]),int(chunks[1])] = float(chunks[2])
        # generate a mask
        self.mask = np.ones(self.mapu.shape, dtype=np.uint8)
        self.mask = cv2.remap(self.mask, self.mapu, self.mapv, cv2.INTER_LINEAR)
        kernel = np.ones((30,30),np.uint8)
        self.mask = cv2.erode(self.mask, kernel, iterations=1)

    """
    Optionally, define a mask
    """
    def set_mask(fmask):
        # add in the additional mask passed in as fmask
        if fmask:
            mask = cv2.cvtColor(cv2.imread(fmask), cv2.COLOR_BGR2GRAY)
            self.mask = self.mask & mask
        new_shape = (int(self.mask.shape[1]*scale), int(self.mask.shape[0]*scale))
        self.mask = cv2.resize(self.mask, new_shape,
                               interpolation=cv2.INTER_CUBIC)
        #plt.figure(1)
        #plt.imshow(self.mask, cmap='gray')
        #plt.show()

    """
    Use OpenCV to undistorted the given image
    """
    def undistort(self, img):
        return cv2.resize(cv2.remap(img, self.mapu, self.mapv, cv2.INTER_LINEAR),
                          (self.mask.shape[1], self.mask.shape[0]),
                          interpolation=cv2.INTER_CUBIC)


def main(args):
    if len(sys.argv) < 3:
        print("Please, specify image folders")
        return 1

    if len(sys.argv) < 4:
        print("Please, specify output rosbag file")
        return 1

    pattern_left = sys.argv[1] + "*.png"
    pattern_right = sys.argv[2] + "*.png"
    cv_bridge = CvBridge()

    file_bag = sys.argv[3]
    bag = rosbag.Bag(file_bag, 'w')

    try:

        for left_img_file in sorted(glob(pattern_left)):

            print("Processing left image: " + left_img_file)

            time_usec = float(path.basename(left_img_file).replace(".png", ""))
            left_img_cv = cv2.imread(left_img_file)

            timestamp = rospy.Time.from_sec(time_usec / 1e9)

            left_img_msg = cv_bridge.cv2_to_imgmsg(left_img_cv, encoding='rgb8')
            left_img_msg.header.stamp = timestamp
            left_img_msg.header.frame_id = "stereo_cam_left"

            bag.write('stereo_cam/left/image_raw', left_img_msg, t=timestamp)

        for right_img_file in sorted(glob(pattern_right)):

            print("Processing right image: " + right_img_file)

            time_usec = float(path.basename(right_img_file).replace(".png", ""))
            right_img_cv = cv2.imread(right_img_file)

            timestamp = rospy.Time.from_sec(time_usec / 1e9)

            right_img_msg = cv_bridge.cv2_to_imgmsg(right_img_cv, encoding='rgb8')
            right_img_msg.header.stamp = timestamp
            right_img_msg.header.frame_id = "stereo_cam_right"

            bag.write('stereo_cam/right/image_raw', right_img_msg, t=timestamp)

    finally:

        bag.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
