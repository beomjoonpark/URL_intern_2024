#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from math import sin, cos, radians
import sys
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
    def __init__(self, name, pos_offset, rpy):
        self.path = Path()
        self.name = name
        self.pos_offset = pos_offset # should be set as (x, y, z) 
        
        cr = cos(0.5*radians(rpy[0]))
        sr = sin(0.5*radians(rpy[0]))
        cp = cos(0.5*radians(rpy[1]))
        sp = sin(0.5*radians(rpy[1]))
        cy = cos(0.5*radians(rpy[2]))
        sy = sin(0.5*radians(rpy[2]))
        self.ori_offset = [
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy
        ] # should be set as (x, y, z, w)
        self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", Odometry, self.gps_callback)
        
        self.odom_pub = rospy.Publisher("/"+name+"/gps_modified/odom", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/"+name+"/gps_modified/pose", PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher("/"+name+"/gps_modified/path", Path, queue_size=10)


    def gps_callback(self, data):
        if self.ori_offset == 0 or self.pos_offset == 0:
            return
        # publishes vins_global

        # (global_position) = (position_offset) + (local_position)*(orientation_offset)
        # (global_orientation) = (orientation_offset)*(local_offset)

        ## pose
        gps_pos = data.pose.pose.position
        gps_ori = [data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w]

        modified_pose = PoseStamped()
        modified_pose.header.stamp = data.header.stamp
        modified_pose.header.frame_id = "world"

        pos_q = [gps_pos.x, gps_pos.y, gps_pos.z, 0]
        rotated_pos = q_mul(q_mul(q_inv(self.ori_offset), pos_q), self.ori_offset)
        modified_pose.pose.position.x = self.pos_offset[0] + rotated_pos[0]
        modified_pose.pose.position.y = self.pos_offset[1] + rotated_pos[1]
        modified_pose.pose.position.z = self.pos_offset[2] + rotated_pos[2]

        # [0]:x, [1]:y, [2]:z, [3]:w
        rotated_ori = q_mul(self.ori_offset, gps_ori)
        modified_pose.pose.orientation.x = rotated_ori[0]
        modified_pose.pose.orientation.y = rotated_ori[1]
        modified_pose.pose.orientation.z = rotated_ori[2]
        modified_pose.pose.orientation.w = rotated_ori[3]

        self.pose_pub.publish(modified_pose)


        ## orientation

        modified_odom = Odometry()
        modified_odom.header.stamp = data.header.stamp
        modified_odom.header.frame_id = "world"
        modified_odom.child_frame_id = "body"

        modified_odom.pose.pose.position.x = self.pos_offset[0] + rotated_pos[0]
        modified_odom.pose.pose.position.y = self.pos_offset[1] + rotated_pos[1]
        modified_odom.pose.pose.position.z = self.pos_offset[2] + rotated_pos[2]
        modified_odom.pose.pose.orientation.x = rotated_ori[0]
        modified_odom.pose.pose.orientation.y = rotated_ori[1]
        modified_odom.pose.pose.orientation.z = rotated_ori[2]
        modified_odom.pose.pose.orientation.w = rotated_ori[3]

        self.odom_pub.publish(modified_odom)


        ## path

        self.path.poses.append(modified_pose)
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "world"
        
        self.path_pub.publish(self.path)
        



''' main '''
if __name__ == '__main__':
    # define drones here
    if len(sys.argv) != 7:
        print("Usage: gps_with_offset.py x y z roll pitch yaw")
        exit(0)
    rospy.init_node("vins_to_gps", anonymous=True)
    xyz = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
    rpy = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]
    d1 = Drone("UAV1", xyz, rpy)
    d2 = Drone("UAV2", xyz, rpy)
    d3 = Drone("UAV3", xyz, rpy)
    d4 = Drone("UAV5", xyz, rpy)
    d5 = Drone("UGV1", xyz, rpy)
    rospy.spin()

