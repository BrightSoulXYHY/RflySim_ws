#!/usr/bin/env python
#!coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()


def callback(img_msg):
    # print("header: seq={img_msg.header.seq} stamp={img_msg.header.stamp} frame_id={img_msg.header.frame_id}")
    # print("width={img_msg.width} height={img_msg.height}")
    print("header: seq={} stamp={} frame_id={}".format(
        img_msg.header.seq, img_msg.header.stamp, img_msg.header.frame_id))
    print("width={} height={}".format(img_msg.width,img_msg.height))
    
    cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv2.imshow("frame" , cv_img)
    cv2.waitKey(1)


def display():
    rospy.init_node('camera_display', anonymous=True)
 
    # rospy.Subscriber('/cam0/image_raw', Image, callback)
    rospy.Subscriber('/camera/left', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    display()
