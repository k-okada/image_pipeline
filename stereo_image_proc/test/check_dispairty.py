#!/usr/bin/env python

import rospy
import cv_bridge
import cv2
from stereo_msgs.msg import DisparityImage

import numpy

cv_bridge = cv_bridge.CvBridge()
def cb(msg):
    im = cv_bridge.imgmsg_to_cv2(msg.image, msg.image.encoding)
    array = numpy.array(im, dtype=numpy.float32)
    print("width = {}, heigt = {}, center depth = {}".format(msg.image.width, msg.image.height, array[msg.image.height/2, msg.image.width/2]))

rospy.init_node('check_disparity')
rospy.Subscriber('stereo/disparity', DisparityImage, cb)
rospy.spin()
