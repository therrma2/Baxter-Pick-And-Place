#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import String
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import UInt16,UInt8
from geometry_msgs.msg import Point

import baxter_interface as bi

global T
T = 0

def node1cb(data, args ):
    global T
    if T == 1:
        return
    bridge = args[0]
    pub = args[1]
    pub2 = args[2]
    try:
       frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)
    #cv2.imshow('frame',frame)
    cv2.waitKey(25)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

     
    hmin = cv2.getTrackbarPos('gHMin','filter')
    smin = cv2.getTrackbarPos('gSMin','filter')
    vmin = cv2.getTrackbarPos('gVMin','filter')

    hmax = cv2.getTrackbarPos('gHMax','filter')
    smax = cv2.getTrackbarPos('gSMax','filter')
    vmax = cv2.getTrackbarPos('gVMax','filter')

    min_green_vales = np.array([hmin,smin,vmin])
    max_green_vales = np.array([hmax,smax,vmax])

    rhmin = cv2.getTrackbarPos('rHMin','filter')
    rsmin = cv2.getTrackbarPos('rSMin','filter')
    rvmin = cv2.getTrackbarPos('rVMin','filter')

    rhmax = cv2.getTrackbarPos('rHMax','filter')
    rsmax = cv2.getTrackbarPos('rSMax','filter')
    rvmax = cv2.getTrackbarPos('rVMax','filter')

    min_red_vales = np.array([rhmin,rsmin,rvmin])
    max_red_vales = np.array([rhmax,rsmax,rvmax])

    maskgreen = cv2.inRange(hsv, min_green_vales, max_green_vales)
    maskred = cv2.inRange(hsv, min_red_vales, max_red_vales)
    #kernel = np.ones((380,380),np.uint8)
    #erosion = cv2.erode(mask,kernel,iterations = 1)#Erode
    #kernel1 = np.ones((380,380),np.uint8)
    #dilation = cv2.dilate(erosion,kernel1,iterations = 1)#Dilate

    maskmain = cv2.addWeighted(maskgreen,0.5,maskred,0.5,0)
    

    cnts = cv2.findContours(maskmain.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        ((x,y),radius) =cv2.minEnclosingCircle(c)
        box = cv2.cv.BoxPoints(rect)
        #print box
        box = np.int0(box)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except:
            return
            #center = 1
            #msg = Point(-500,-500,-500)
            
        #print "--------------------------------------------------------"

        #print "box0", box[0]
        #print "box1", box[1]
        #print "box2", box[2]
        #print "box3", box[3]
        #if center != 1:
        if radius > 20:
            cv2.drawContours(frame,[box],0,(0,255,255),2)
            cv2.circle(maskmain, center, 5, (0, 0, 255), -1)

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
        msg2 = bridge.cv2_to_imgmsg(frame,encoding = "bgr8")
        pub.publish(msg)
        pub2.publish(msg2)    
        

    cv2.imshow('dilation',maskmain)
    cv2.imshow('newframe',frame)
    
    cv2.waitKey(25)

def nothing(self):
    pass


def smilecb(data,pub2):
    global T
    T = 1
    img = cv2.imread('Smiley.jpg')
    bridge2 = CvBridge()
    msg = bridge2.cv2_to_imgmsg(img, encoding="bgr8")
    rospy.sleep(.2)
    pub2.publish(msg)
    print "callback triggered"
    sys.exit("Yay!")


def node1():
    
    rospy.init_node("rcam")
    bridge = CvBridge()
    rightcam = bi.CameraController('right_hand_camera')
        
    bridge = CvBridge()

    cv2.namedWindow ('filter')

    cv2.createTrackbar('gHMin','filter',31,255,nothing)
    cv2.createTrackbar('gSMin','filter',85,255,nothing)
    cv2.createTrackbar('gVMin','filter',118,255,nothing)

    cv2.createTrackbar('gHMax','filter',107,255,nothing)
    cv2.createTrackbar('gSMax','filter',255,255,nothing)
    cv2.createTrackbar('gVMax','filter',255,255,nothing)

    cv2.createTrackbar('rHMin','filter',0,255,nothing)
    cv2.createTrackbar('rSMin','filter',81,255,nothing)
    cv2.createTrackbar('rVMin','filter',54,255,nothing)

    cv2.createTrackbar('rHMax','filter',179,255,nothing)
    cv2.createTrackbar('rSMax','filter',234,255,nothing)
    cv2.createTrackbar('rVMax','filter',255,255,nothing)


    pub = rospy.Publisher('rcampub', Point, queue_size=10)
    pub2 = rospy.Publisher('/robot/xdisplay',Image,queue_size=10)

    rospy.Subscriber('/cameras/right_hand_camera/image', Image, node1cb, callback_args=(bridge, pub,pub2))

    rospy.Subscriber('smile',UInt8,smilecb,pub2)
    
    print 'here before spin'
    rospy.spin()

    
node1()
