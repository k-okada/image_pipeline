#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Kei Okada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Kei OKada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import cv2
import math

import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

def pub_stereo_image():
    bridge = CvBridge()
    rospy.init_node('pub_stereo_image', anonymous=True)
    pub_img_r = rospy.Publisher('right/image_raw', Image, queue_size=1)
    pub_img_l = rospy.Publisher('left/image_raw', Image, queue_size=1)
    pub_info_r = rospy.Publisher('right/camera_info', CameraInfo, queue_size=1)
    pub_info_l = rospy.Publisher('left/camera_info', CameraInfo, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    # image settings
    width = 320
    height = 240
    image_l = np.random.randint(0, 255, (height, width, 1), np.uint8)
    # artificial disparities
    far  = 10
    near = 20
    image_r = np.roll(image_l, -far, axis=1)
    image_r[height/3:height*2/3,width/3:width*2/3] = image_l[height/3:height*2/3,width/3+near:width*2/3+near]

    msg_info_r = CameraInfo()
    msg_info_l = CameraInfo()
    msg_info_r.header.frame_id = 'camera_l'
    msg_info_l.header.frame_id = 'camera_l'
    msg_info_r.height = msg_info_l.height = height
    msg_info_r.width  = msg_info_l.width  = width
    msg_info_r.distortion_model = msg_info_l.distortion_model = 'plumb_bob'
    msg_info_r.D = msg_info_l.D = [0,0,0,0,0]
    msg_info_r.K = msg_info_l.K = [width,      0,  width/2,
                                       0, height, height/2,
                                       0,      0,        1]
    msg_info_r.R = msg_info_l.R = np.identity(3).reshape(-1).tolist()
    msg_info_l.P = [width,      0,  width/2, 0,
                        0, height, height/2, 0,
                        0,      0,        1, 0]
    msg_info_r.P = np.copy(msg_info_l.P)
    msg_info_r.P[3] = -50

    t = 0
    while not rospy.is_shutdown():
        cx_offset = int(math.fabs(round(10*(math.cos(t)))))
        t = t + 0.01
        #
        # update camera info
        msg_info_r.P[2] = width/2 + cx_offset
        # update image
        far =  10 - cx_offset
        near = 20 - cx_offset
        image_r = np.roll(image_l, -far, axis=1)
        image_r[height/3:height*2/3,width/3:width*2/3] = image_l[height/3:height*2/3,width/3+near:width*2/3+near]
        #
        rospy.loginfo("publish images with cx offset %d"%(cx_offset))
        stamp = rospy.Time.now()
        msg_img_r = bridge.cv2_to_imgmsg(image_r, "mono8")
        msg_img_r.header.frame_id = 'camera_r'
        msg_img_r.header.stamp = stamp
        msg_img_l = bridge.cv2_to_imgmsg(image_l, "mono8")        
        msg_img_l.header.frame_id = 'camera_l'
        msg_img_l.header.stamp = stamp
        pub_img_r.publish(msg_img_r)
        pub_img_l.publish(msg_img_l)
        msg_info_r.header.stamp = stamp
        msg_info_l.header.stamp = stamp
        pub_info_r.publish(msg_info_r)
        pub_info_l.publish(msg_info_l)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_stereo_image()
    except rospy.ROSInterruptException:
        pass
