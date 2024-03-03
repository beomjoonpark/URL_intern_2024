#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

#import sys
# import signal

# def signal_handler(signal, frame): # ctrl + c -> exit program
#         print('You pressed Ctrl+C!')
#         sys.exit(0)
# signal.signal(signal.SIGINT, signal_handler)


''' class '''

def q_mul(q1, q2):
    a1, b1, c1, d1 = q1
    a2, b2, c2, d2 = q2
    return [a1*a2-b1*b2-c1*c2-d1*d2, 
            a1*b2+b1*a2+c1*d2-d1*c2,
            a1*c2-b1*d2+c1*a2+d1*b2,
            a1*d2+b1*c2-c1*b2+d1*a2]
def q_inv(a):
    r0, r1, r2, r3 = a
    return [r0, -r1, -r2, -r3]

class Pose:
    def __init__(self, p, q):
        self.p = p
        self.q = q

    def inv(self):
        newq = q_inv(self.q)
        newp = q_inv(q_mul(q_mul(newq, self.p), self.q))
        return Pose(newp, newq)
    
    def mul(self, b):
        newp = [self.p[i] + q_mul(q_mul(self.q, b.p), q_inv(self.q))[i] for i in range(4)]
        newq = q_mul(self.q, b.q)
        return Pose(newp, newq)
    

# exp2    
# pose1 = Pose([0, -0.516735165739, 0.976238481298, 1.98070287584],
#              [0.684428422397, 0.0154627274378, -0.0511488282603, 0.727119272227])
# pose2 = Pose([0, 6.39268102532, 1.02783979016, 2.23825520218],
#              [0.632393990086, -0.00370971732458,-0.0133746993795, 0.774522560496])
# pose3 = Pose([0, -5.12725397859, -0.849855804721, 2.40667665611],
#              [0.993003357854, -0.0268209676436, 0.0274246563793, 0.111681937695])
# pose4 = Pose([0, 12.9589817894, 1.99550177283, 2.09574845195],
#              [0.997874820108, -0.0187317778608, 0.0352527411414, 0.051499593543])
# pose5 = Pose([0, -11.1565039823, -2.1310370952, 2.37392776969],
#              [0.821548385625, -0.0247007139164, -0.00284691051934, 0.569596365779])

# exp2 first version
# pose1 = Pose([0, 0.0157845869245, -0.0380966746825, 0.00754460745462],
#              [0.745663177569, -0.0550371492699, 0.0171038786796, 0.663825877133])
# pose2 = Pose([0, 6.11818831833, 0.0414924711544, -0.000824401605102],
#              [0.826295297204, -0.00915304728842, -0.0108361332251, 0.5633058506516])
# pose3 = Pose([0, -5.61373092716, -1.77929691371, -0.0258456839297],
#              [0.858187076394, -0.0377148526309, 0.0143744534202, 0.511747877109])
# pose4 = Pose([0, 11.6426213758, 1.96266547803, 0.0416028197743],
#              [0.998116572394, -0.0179693165204, 0.0346305238202, 0.047340663242])
# pose5 = Pose([0, -11.8635327942, -3.06168434928, -0.049968418659],
#              [0.812493279769, -0.0217623578867, -0.0114757315749, 0.582451180525])



# exp1
# pose1 = Pose([0, -0.71513526838, 1.1370034004, 2.52149896099],
#              [0.580915166529, -0.0370105137185, -0.0115137427984, 0.813040727699])
# pose2 = Pose([0, 24.2855716096, -0.853136989037, 0.794241096588],
#              [0.989253359053, -0.0386210386352, -0.0155957124283, 0.140153418549])
# pose3 = Pose([0, 8.7821934059, -14.459374253, 3.00636175731],
#              [0.0508044623346, -0.0121358231235, 0.0259463174782, -0.998297759696])
# pose4 = Pose([0, 23.9433733863, 13.766164716, 5.35108296661],
#              [0.919758997664, -0.01580876859, -0.0357113321446, 0.390535747159])
# pose5 = Pose([0, 33.9428608361, -6.16427488958, -0.265278262625],
#              [0.893079694648, -0.0167882646152, 0.0189536241929, 0.449185455363])



class Drone:
    def __init__(self, name):
        self.name = name

        self.gps_init = 0
        self.vio_init = 0

        self.gps_p = 0
        self.gps_q = 0
        self.vio_p = 0
        self.vio_q = 0

        self.gps_pl = 0
        self.gps_ql = 0
        self.vio_pl = 0
        self.vio_ql = 0
        
        # self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", Odometry, self.gps_callback)
        self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", PoseStamped, self.gps_callback)

        self.vins_sub = rospy.Subscriber("/"+name+"/vins_estimator/odometry", Odometry, self.vins_callback)


    # def gps_callback(self, data):
    #     if self.gps_init == 0:
    #         self.gps_p = (0, data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    #         self.gps_q = (data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
            
    #         self.gps_init = 1
    #     else:
    #         self.gps_pl = (0, data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    #         self.gps_ql = (data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)

    def gps_callback(self, data):
        if self.gps_init == 0:
            self.gps_p = (0, data.pose.position.x, data.pose.position.y, data.pose.position.z)
            self.gps_q = (data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
            
            self.gps_init = 1
        else:
            self.gps_pl = (0, data.pose.position.x, data.pose.position.y, data.pose.position.z)
            self.gps_ql = (data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)


    def vins_callback(self, data):
        if self.vio_init == 0:
            self.vio_p = (0, data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
            self.vio_q = (data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)
            
            self.vio_init = 1
        else: 
            self.vio_pl = (0, data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
            self.vio_ql = (data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)



''' main '''
if __name__ == '__main__':
    # define drones here
    rospy.init_node("vins_to_gps", anonymous=True)
    # d1 = Drone("UAV1")
    # d2 = Drone("UAV2")
    # d3 = Drone("UAV3")
    # d4 = Drone("UAV5")
    # d5 = Drone("UGV1")

    d1 = Drone("UAV1")
    d2 = Drone("UAV2")
    d3 = Drone("UAV3")
    d4 = Drone("UAV4")
    d5 = Drone("UAV5")

    dronelist = [d1, d2, d3, d4, d5]
    rospy.spin()


    i = 0
    for d in dronelist:

        print(d.name)
        # vins_pose = Pose(d.vio_p, d.vio_q)
        # offset = gps_pose.mul(vins_pose.inv())
        
        # gps_vec = [d.gps_pl[i] - d.gps_p[i] for i in range(4)]
        # vio_vec = [d.vio_pl[i] - d.vio_p[i] for i in range(4)]

        # print(offset.p)
        # print(offset.q)
        # print(gps_vec)
        # print(vio_vec)
        print(d.gps_p)
        print(d.gps_q)
        # print(d.vio_p)
        # print(d.vio_q)
        print("")
        i = i+1

    

    

