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
from geometry_msgs.msg import Point

import baxter_interface as bi


def node1cb(data, args ):
    bridge = args[0]
    pub = args[1]
    try:
       frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)
    #cv2.imshow('frame',frame)
    cv2.waitKey(25)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

     
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
        rect = cv2.minAreaRect(c)
        ((x,y),radius) =cv2.minEnclosingCircle(c)
        box = cv2.cv.BoxPoints(rect)
        print box
        box = np.int0(box)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisonError:
            return
        #print "--------------------------------------------------------"

        #print "box0", box[0]
        #print "box1", box[1]
        #print "box2", box[2]
        #print "box3", box[3]

        if radius > 20:
            cv2.drawContours(frame,[box],0,(0,255,255),2)
            cv2.circle(mask, center, 5, (0, 0, 255), -1)

        box2 = [[box[0][0],box[0][1]],[box[1][0],box[1][1]],[box[2][0],box[2][1]],[box[3][0],box[3][1]]]
        #print box2
        boxx = [box2[0][0],box2[1][0],box2[2][0],box2[3][0]]
        boxy = [box2[0][1],box2[1][1],box2[2][1],box2[3][1]]
        
        index1 = boxy.index(max(boxy))
        
        pt1 = box2.pop(index1)
        #print "pt1", pt1
        boxx = [box2[0][0],box2[1][0],box2[2][0]]
        boxy = [box2[0][1],box2[1][1],box2[2][1]]
        
        index2 = boxy.index(max(boxy))

        pt2 = box2.pop(index2)
        #print "pt2",pt2
        #print(x,y)
        
        pt1 = (pt1[0],pt1[1])
        pt2 = (pt2[0],pt2[1])

        cv2.line(frame,pt1,pt2,(0,0,255),thickness=3)
        slope = float((float(pt2[1])-float(pt1[1]))/(float(pt2[0])-float(pt1[0])))
        
        x_int = int(x)
        y_int = int(y)
        
        msg = Point(x,y,slope)

        pub.publish(msg)    
        

    cv2.imshow('dilation',mask)
    cv2.imshow('newframe',frame)
    
    cv2.waitKey(25)

def nothing(self):
    pass


def node1():
    
    rospy.init_node("rcam")
    bridge = CvBridge()
    rightcam = bi.CameraController('right_hand_camera')
        
    bridge = CvBridge()

    cv2.namedWindow ('filter')

    cv2.createTrackbar('HMin','filter',31,255,nothing)
    cv2.createTrackbar('SMin','filter',85,255,nothing)
    cv2.createTrackbar('VMin','filter',118,255,nothing)

    cv2.createTrackbar('HMax','filter',107,255,nothing)
    cv2.createTrackbar('SMax','filter',255,255,nothing)
    cv2.createTrackbar('VMax','filter',255,255,nothing)


    pub = rospy.Publisher('rcampub', Point, queue_size=10)
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, node1cb, callback_args=(bridge, pub))
    
    print 'here before spin'
    rospy.spin()

    
node1()
