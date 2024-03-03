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

def q_mul(a, b):
    r1, r2, r3, r0 = a
    s1, s2, s3, s0 = b
    return [r0*s1+r1*s0-r2*s3+r3*s2, 
            r0*s2+r1*s3+r2*s0-r3*s1,
            r0*s3-r1*s2+r2*s1+r3*s0,
            r0*s0-r1*s1-r2*s2-r3*s3]
def q_inv(a):
    r1, r2, r3, r0 = a
    return [-r1, -r2, -r3, r0]



class Drone:
    def __init__(self, name):
        self.path = Path()
        self.name = name
        self.pos_offset = 0 # should be set as (x, y, z) 
        self.ori_offset = 0 # should be set as (x, y, z, w)

        self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", PoseStamped, self.gps_callback)
        self.vins_sub = rospy.Subscriber("/"+name+"/vins_estimator/odometry", Odometry, self.vins_callback)

        self.odom_pub = rospy.Publisher("/"+name+"/vins_global/odometry", Odometry, queue_size=100)
        self.pose_pub = rospy.Publisher("/"+name+"/vins_global/pose", PoseStamped, queue_size=100)
        self.path_pub = rospy.Publisher("/"+name+"/vins_global/path", Path, queue_size=100)


    def gps_callback(self, data):
        if self.pos_offset == 0:
            self.pos_offset = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
            self.ori_offset = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
            print(self.name, self.pos_offset, self.ori_offset)
        else: return


    def vins_callback(self, data):
        if self.ori_offset == 0 or self.pos_offset == 0:
            return
        # publishes vins_global

        # (global_position) = (position_offset) + (local_position)*(orientation_offset)
        # (global_orientation) = (orientation_offset)*(local_offset)

        ## pose
        vins_pos = data.pose.pose.position
        vins_ori = [data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w]

        vins_global_pose = PoseStamped()
        vins_global_pose.header.stamp = data.header.stamp
        vins_global_pose.header.frame_id = "world"

        pos_q = [vins_pos.x, vins_pos.y, vins_pos.z, 0]
        rotated_pos = q_mul(q_mul(q_inv(self.ori_offset), pos_q), self.ori_offset)
        vins_global_pose.pose.position.x = self.pos_offset[0] + rotated_pos[0]
        vins_global_pose.pose.position.y = self.pos_offset[1] + rotated_pos[1]
        vins_global_pose.pose.position.z = self.pos_offset[2] + rotated_pos[2]

        # [0]:x, [1]:y, [2]:z, [3]:w
        rotated_ori = q_mul(self.ori_offset, vins_ori)
        vins_global_pose.pose.orientation.x = rotated_ori[0]
        vins_global_pose.pose.orientation.y = rotated_ori[1]
        vins_global_pose.pose.orientation.z = rotated_ori[2]
        vins_global_pose.pose.orientation.w = rotated_ori[3]

        self.pose_pub.publish(vins_global_pose)


        ## orientation

        vins_global_odom = Odometry()
        vins_global_odom.header.stamp = data.header.stamp
        vins_global_odom.header.frame_id = "world"
        vins_global_odom.child_frame_id = "body"

        vins_global_odom.pose.pose.position.x = self.pos_offset[0] + rotated_pos[0]
        vins_global_odom.pose.pose.position.y = self.pos_offset[1] + rotated_pos[1]
        vins_global_odom.pose.pose.position.z = self.pos_offset[2] + rotated_pos[2]
        vins_global_odom.pose.pose.orientation.x = rotated_ori[0]
        vins_global_odom.pose.pose.orientation.y = rotated_ori[1]
        vins_global_odom.pose.pose.orientation.z = rotated_ori[2]
        vins_global_odom.pose.pose.orientation.w = rotated_ori[3]

        self.odom_pub.publish(vins_global_odom)


        ## path

        self.path.poses.append(vins_global_pose)
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        
        self.path_pub.publish(self.path)



''' main '''
if __name__ == '__main__':
    # define drones here
    rospy.init_node("vins_to_gps", anonymous=True)
    d1 = Drone("UAV1")
    d2 = Drone("UAV2")
    d3 = Drone("UAV3")
    d4 = Drone("UAV5")
    d5 = Drone("UGV1")
    rospy.spin()

