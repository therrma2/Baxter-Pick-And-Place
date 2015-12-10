#!/usr/bin/env python

import rospy
import baxter_interface
import sys
import struct
import threading 
from math import  atan2

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from std_msgs.msg import UInt8
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from sensor_msgs.msg import (JointState,
)

from baxter_interface import CHECK_VERSION

import numpy as np
##################################

# Global Variables



##########################
def ik_solve(limb,p,q):
    #rospy.init_node("node1")
    global right
    global left
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
    try:
        return limb_joints
    except:
        mess = 1
        smile.publish(mess)
        left.move_to_joint_positions({'left_s0': -0.7597039843322755, 'left_s1': -0.9407137170959473, 'left_w0': 0.7942185520202637, 'left_w1': -0.06596117380371094, 'left_w2': -1.8983012228393557, 'left_e0': 2.859723680548096, 'left_e1': 0.1580000209716797})
        right.move_to_joint_positions({'right_s0': -0.7597039843322755, 'right_s1': -0.9407137170959473, 'right_w0': 0.7942185520202637, 'right_w1': -0.06596117380371094, 'right_w2': -1.8983012228393557, 'right_e0': 2.859723680548096, 'right_e1': 0.1580000209716797})
        sys.exit("All Done!")
        

def camcb(data):
    global camdata
    camdata = data
    #print camdata



def main():
    rospy.init_node('movenode')
    rospy.Subscriber('rcampub',Point,camcb)
    rospy.sleep(.2)
    global camdata
    #print "camdata is" ,camdata
    global right
    global rg
    global left
    global smile
    smile = rospy.Publisher("smile",UInt8,queue_size=10)

    right = baxter_interface.Limb('right')
    right.set_joint_position_speed(1.0)
    left = baxter_interface.Limb('left')
    left.set_joint_position_speed(1.0)
    camdata = Point()
    rg = baxter_interface.Gripper('right')
    
    yfactor = .000667917
    xfactor = .0008135938

    if rg.calibrated() == False:
        rg.calibrate()
    #rg.calibrate()
    print right.endpoint_pose()
    print right.joint_angles()
    Q = Quaternion(x=0.7523426889287905, y=-0.6584930265055371, z=0.0010142237493953393, w=0.019141154854433382)
    S1p = Point(x=.55,y=-.569769,z=.12)
    S2p = Point(.70,-.2651429,.12)
    S3p = Point(.55,.00738,.12)

    S1j = ik_solve('right',S1p,Q)
    S2j = ik_solve('right',S2p,Q)
    S3j = ik_solve('right',S3p,Q)   
    
    #Dp = Point(x=0.5243308743510079, y=0.34035668580244005, z=0.20640086748510136)
    Dp = Point(x=0.5719869428372248, y=-1.0075248563301191, z=0.13966290441126045)
    Dq = Quaternion(x=-0.5072676977914469, y=0.8364736871351672, z=-0.20583706250527933, w=-0.024947088147947636)
    Dj = {'right_e0': 1.4193157223693849, 'right_e1': 0.7152185415344239, 'right_s0': 1.2824079372070314, 'right_s1': -0.23853401224365237, 'right_w0': -1.2916118219238282, 'right_w1': 1.645961383520508, 'right_w2': -0.3512816000244141}


    for j in range(6):

        right.move_to_joint_positions(S1j)
    
        i=0
        #move on top of block
        while i <2:
            current = camdata
            print camdata
            cpose = Point(right.endpoint_pose()['position'].x,right.endpoint_pose()['position'].y,right.endpoint_pose()['position'].z)

            xoffsetpix = current.x-330
            yoffsetpix = 190-current.y


            xoffsetm = xoffsetpix*xfactor
            yoffsetm = yoffsetpix*yfactor
            goal = Point(xoffsetm+cpose.x,yoffsetm+cpose.y,cpose.z)

            right.move_to_joint_positions(ik_solve('right',goal,Q))
            #rospy.sleep(.15)

            i = i+1

       
        #move down to z= 0
        goal.z = 0
        right.move_to_joint_positions(ik_solve('right',goal,Q))
        
        #iterate again
        




        slope = camdata.z
        theta = atan2(slope,1)

        cang = right.joint_angles()
        cang['right_w2'] = cang['right_w2']+theta



        right.move_to_joint_positions(cang)
        
        Fq = right.endpoint_pose()['orientation']

        goal.z = -.1
        right.move_to_joint_positions(ik_solve('right',goal,Fq))

        rg.close()
        rospy.sleep(.05)
        goal.z = .12
        right.move_to_joint_positions(ik_solve('right',goal,Q))
        right.move_to_joint_positions(ik_solve('right',Dp,Dq))
        rg.open()

    mess = 1

    left.move_to_joint_positions({'left_s0': -0.7597039843322755, 'left_s1': -0.9407137170959473, 'left_w0': 0.7942185520202637, 'left_w1': -0.06596117380371094, 'left_w2': -1.8983012228393557, 'left_e0': 2.859723680548096, 'left_e1': 0.1580000209716797})
    right.move_to_joint_positions({'right_s0': -0.7597039843322755, 'right_s1': -0.9407137170959473, 'right_w0': 0.7942185520202637, 'right_w1': -0.06596117380371094, 'right_w2': -1.8983012228393557, 'right_e0': 2.859723680548096, 'right_e1': 0.1580000209716797})

    smile.publish(mess)
    sys.exit("All Done!")    















if __name__ == '__main__':
    
    main()
