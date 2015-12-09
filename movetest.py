#!/usr/bin/env python


import rospy
import struct
import sys
import threading 

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from sensor_msgs.msg import (JointState,
)


import numpy as np

import baxter_interface 

from baxter_interface import CHECK_VERSION


def ik_solve(limb,p,q):
    rospy.init_node("node1")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    
    
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    poses = {
        'left': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q)),
        'right': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q))        
    }
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled 
    
    right = baxter_interface.Limb('right')
   
    
    
  
    
    '''seeds={
         'right': JointState(
             header=hdr,
             name= right.joint_angles().keys(),
             position= right.joint_angles().values()
             )
             }
    
    #using random seeds  
    
    seeds2={
         'right': JointState(
             header=hdr,
             name= ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1'],

             position= [-1.3483691110107423, 
             -0.11850001572875977, 1.18768462366333, -0.002300971179199219,0.4371845240478516, 
             1.8419274289489747, 0.4981602602966309]
             )
             }
             
    '''
    
    ikreq.pose_stamp.append(poses[limb])
    
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
            
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    for i in range(50):
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  #(seed_str,))
        # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            #print "\nIK Joint Solution:\n", limb_joints
            #print "------------------"
            #print "Response Message:\n", resp
            break
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            
            noise= np.random.normal(0,0.25,7)
            
            '''print right.joint_angles().values()
            print right.joint_angles().keys()
            print right.joint_angles()
            print "\r\n"
            '''
            
            '''seeds_random={
                'right': JointState(
                 header=hdr,
                 name= right.joint_angles().keys(),
                 position= (right.joint_angles().values() + noise).tolist() 
                 )
             }
             '''
            
            js = JointState()
            js.header = hdr 
            i = 0
            for key,val in right.joint_angles().iteritems():
                js.name.append(key)
                js.position.append(val+noise[i])
                i += 1
           
            
            ikreq.seed_angles = [js]
            
            resp = iksvc(ikreq)

    return limb_joints
