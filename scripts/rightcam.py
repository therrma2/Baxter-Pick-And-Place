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

    rospy.init_node("rightcam")
    bridge = CvBridge()
    cv2.namedWindow('rightcam')
    headcam = bi.CameraController('right_hand_camera')
    
    rospy.Subscriber('/cameras/right_hand_camera/image',Image,cb,callback_args=bridge)
    print 'here before spin'
    rospy.spin()


    
main()
