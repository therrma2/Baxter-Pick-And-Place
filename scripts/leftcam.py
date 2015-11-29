#!/usr/bin/env python

import rospy
import baxter_interface as bi
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def cb(data,bridge):
    try:
        frame = bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError,e:
        print("==[CAMERA MANAGER]==",e)
    cv2.imshow('frame',frame)
    cv2.waitKey(25)


def main():

    rospy.init_node("leftcam")
    bridge = CvBridge()
    cv2.namedWindow('leftcam')
    headcam = bi.CameraController('left_hand_camera')
    
    rospy.Subscriber('/cameras/left_hand_camera/image',Image,cb,callback_args=bridge)
    print 'here before spin'
    rospy.spin()


    
main()
