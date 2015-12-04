#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import String
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import UInt16

import baxter_interface as bi



#def cb(data,bridge):
#    try:
#        frame = bridge.imgmsg_to_cv2(data,"bgr8")
#    except CvBridgeError,e:
#        print("==[CAMERA MANAGER]==",e)
#    cv2.imshow('frame',frame)
#    cv2.waitKey(25)

def node1cb(data, args ):
    #rospy.loginfo("new image")
    bridge = args[0]
    pub = args[1]
    try:
       frame1 = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)
    cv2.imshow('frame',frame1)
    cv2.waitKey(25)

    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
    

     
    hmin = cv2.getTrackbarPos('HMin','filter')
    smin = cv2.getTrackbarPos('SMin','filter')
    vmin = cv2.getTrackbarPos('VMin','filter')

    hmax = cv2.getTrackbarPos('HMax','filter')
    smax = cv2.getTrackbarPos('SMax','filter')
    vmax = cv2.getTrackbarPos('VMax','filter')

    min_red_vales = np.array([hmin,smin,vmin])
    max_red_vales = np.array([hmax,smax,vmax])

    mask = cv2.inRange(hsv, min_red_vales, max_red_vales)
    kernel = np.ones((380,380),np.uint8)
    erosion = cv2.erode(mask,kernel,iterations = 1)#Erode
    kernel1 = np.ones((380,380),np.uint8)
    dilation = cv2.dilate(erosion,kernel1,iterations = 1)#Dilate

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisonError:
            return

        
        #print "first if"
        if radius > 20:
            #print "Radius =",radius
            #print 'second if'
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(mask, (int(x), int(y)), int(radius),(0, 255, 255), 5 )
            cv2.circle(mask, center, 5, (0, 0, 255), -1)

        print(x,y)

        x_int = int(x)
        y_int = int(y)
        msg = UInt16()
        msg.data = x_int
        pub.publish(msg)    
        

    
    cv2.imshow('cv_image',mask)


       
    cv2.imshow('frame1',frame1)
    cv2.imshow('dilation',mask)
    
    cv2.waitKey(25)

def nothing(self):
    pass


def node1():
    
    rospy.init_node("rightcam")
    bridge = CvBridge()
    rightcam = bi.CameraController('right_hand_camera')
    
    #rospy.Subscriber('/cameras/right_hand_camera/image',Image,cb,callback_args=bridge)
    
    
    #rospy.init_node('node1', anonymous=True)
    
    bridge = CvBridge()

    cv2.namedWindow ('filter')

    cv2.createTrackbar('HMin','filter',22,255,nothing)
    cv2.createTrackbar('SMin','filter',78,255,nothing)
    cv2.createTrackbar('VMin','filter',132,255,nothing)

    cv2.createTrackbar('HMax','filter',200,255,nothing)
    cv2.createTrackbar('SMax','filter',194,255,nothing)
    cv2.createTrackbar('VMax','filter',228,255,nothing)


    pub = rospy.Publisher('node1pub', UInt16, queue_size=10)
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, node1cb, callback_args=(bridge, pub))

    
    print 'here before spin'
    rospy.spin()

    
node1()
