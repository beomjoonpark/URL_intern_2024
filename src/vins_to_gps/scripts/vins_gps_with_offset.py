#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from math import sin, cos, radians
import sys



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
    def __init__(self, name, xyz, rpy, exp):
        self.gps_path = Path()
        self.vins_path = Path()
        self.vinsmodi_path = Path()
        self.name = name
        self.vins_pos_offset = 0 # should be set as (x, y, z) 
        self.vins_ori_offset = 0 # should be set as (x, y, z, w)
        self.exp = exp

        self.world_pos_offset = xyz
        cr = cos(0.5*radians(rpy[0]))
        sr = sin(0.5*radians(rpy[0]))
        cp = cos(0.5*radians(rpy[1]))
        sp = sin(0.5*radians(rpy[1]))
        cy = cos(0.5*radians(rpy[2]))
        sy = sin(0.5*radians(rpy[2]))
        self.world_ori_offset = [
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy
        ]      
        if exp==1:
            self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", Odometry, self.gps_callback)
        else:
            self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", PoseStamped, self.gps_callback)


        # self.vins_sub = rospy.Subscriber("/"+name+"/vins_global/odometry", Odometry, self.vins_callback)
        
        # self.vinsmodi_odom_pub = rospy.Publisher("/"+name+"/vins_modified/odometry", Odometry, queue_size=10)
        # self.vinsmodi_pose_pub = rospy.Publisher("/"+name+"/vins_modified/pose", PoseStamped, queue_size=10)
        # self.vinsmodi_path_pub = rospy.Publisher("/"+name+"/vins_modified/path", Path, queue_size=10)
        
        # self.vins_odom_pub = rospy.Publisher("/"+name+"/vins_modified/odometry", Odometry, queue_size=10)
        # self.vins_pose_pub = rospy.Publisher("/"+name+"/vins_modified/pose", PoseStamped, queue_size=10)
        # self.vins_path_pub = rospy.Publisher("/"+name+"/vins_modified/path", Path, queue_size=10)

        self.gps_odom_pub = rospy.Publisher("/"+name+"/gps_modified/odom", Odometry, queue_size=10)
        self.gps_pose_pub = rospy.Publisher("/"+name+"/gps_modified/pose", PoseStamped, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/"+name+"/gps_modified/path", Path, queue_size=10)


    def gps_callback(self, data):
        # if self.vins_pos_offset == 0:
        #     self.vins_pos_offset = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        #     self.vins_ori_offset = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            # self.vins_pos_offset = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
            # self.vins_ori_offset = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        
        if self.exp ==1:
            gps_pos = data.pose.pose.position
            gps_ori = [data.pose.pose.orientation.x,
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w]
        else:
            gps_pos = data.pose.position
            gps_ori = [data.pose.orientation.x,
                        data.pose.orientation.y,
                        data.pose.orientation.z,
                        data.pose.orientation.w]

        modified_pose = PoseStamped()
        modified_pose.header.stamp = data.header.stamp
        modified_pose.header.frame_id = "world"

        pos_q = [gps_pos.x, gps_pos.y, gps_pos.z, 0]
        rotated_pos = q_mul(q_mul(q_inv(self.world_ori_offset), pos_q), self.world_ori_offset)
        modified_pose.pose.position.x = self.world_pos_offset[0] + rotated_pos[0]
        modified_pose.pose.position.y = self.world_pos_offset[1] + rotated_pos[1]
        modified_pose.pose.position.z = self.world_pos_offset[2] + rotated_pos[2]

        # [0]:x, [1]:y, [2]:z, [3]:w
        rotated_ori = q_mul(self.world_ori_offset, gps_ori)
        modified_pose.pose.orientation.x = rotated_ori[0]
        modified_pose.pose.orientation.y = rotated_ori[1]
        modified_pose.pose.orientation.z = rotated_ori[2]
        modified_pose.pose.orientation.w = rotated_ori[3]

        self.gps_pose_pub.publish(modified_pose)


        ## odometry

        modified_odom = Odometry()
        modified_odom.header.stamp = data.header.stamp
        modified_odom.header.frame_id = "world"
        modified_odom.child_frame_id = "body"

        modified_odom.pose.pose.position.x = self.world_pos_offset[0] + rotated_pos[0]
        modified_odom.pose.pose.position.y = self.world_pos_offset[1] + rotated_pos[1]
        modified_odom.pose.pose.position.z = self.world_pos_offset[2] + rotated_pos[2]
        modified_odom.pose.pose.orientation.x = rotated_ori[0]
        modified_odom.pose.pose.orientation.y = rotated_ori[1]
        modified_odom.pose.pose.orientation.z = rotated_ori[2]
        modified_odom.pose.pose.orientation.w = rotated_ori[3]

        self.gps_odom_pub.publish(modified_odom)


        ## path

        self.gps_path.poses.append(modified_pose)
        self.gps_path.header.stamp = rospy.Time.now()
        self.gps_path.header.frame_id = "world"
        
        self.gps_path_pub.publish(self.gps_path)

    def vins_callback(self, data):
        # if self.vins_pos_offset == 0:
        #     self.vins_pos_offset = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        #     self.vins_ori_offset = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            # self.vins_pos_offset = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
            # self.vins_ori_offset = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        
        vins_pos = data.pose.pose.position
        vins_ori = [data.pose.pose.orientation.x,
                    data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w]
        
        # gps_pos = data.pose.position
        # gps_ori = [data.pose.orientation.x,
        #             data.pose.orientation.y,
        #             data.pose.orientation.z,
        #             data.pose.orientation.w]

        modified_pose = PoseStamped()
        modified_pose.header.stamp = data.header.stamp
        modified_pose.header.frame_id = "world"

        pos_q = [vins_pos.x, vins_pos.y, vins_pos.z, 0]
        rotated_pos = q_mul(q_mul(q_inv(self.world_ori_offset), pos_q), self.world_ori_offset)
        modified_pose.pose.position.x = self.world_pos_offset[0] + rotated_pos[0]
        modified_pose.pose.position.y = self.world_pos_offset[1] + rotated_pos[1]
        modified_pose.pose.position.z = self.world_pos_offset[2] + rotated_pos[2]

        # [0]:x, [1]:y, [2]:z, [3]:w
        rotated_ori = q_mul(self.world_ori_offset, vins_ori)
        modified_pose.pose.orientation.x = rotated_ori[0]
        modified_pose.pose.orientation.y = rotated_ori[1]
        modified_pose.pose.orientation.z = rotated_ori[2]
        modified_pose.pose.orientation.w = rotated_ori[3]

        self.vins_pose_pub.publish(modified_pose)


        ## odometry

        modified_odom = Odometry()
        modified_odom.header.stamp = data.header.stamp
        modified_odom.header.frame_id = "world"
        modified_odom.child_frame_id = "body"

        modified_odom.pose.pose.position.x = self.world_pos_offset[0] + rotated_pos[0]
        modified_odom.pose.pose.position.y = self.world_pos_offset[1] + rotated_pos[1]
        modified_odom.pose.pose.position.z = self.world_pos_offset[2] + rotated_pos[2]
        modified_odom.pose.pose.orientation.x = rotated_ori[0]
        modified_odom.pose.pose.orientation.y = rotated_ori[1]
        modified_odom.pose.pose.orientation.z = rotated_ori[2]
        modified_odom.pose.pose.orientation.w = rotated_ori[3]

        self.vins_odom_pub.publish(modified_odom)


        ## path

        self.vins_path.poses.append(modified_pose)
        self.vins_path.header.stamp = rospy.Time.now()
        self.vins_path.header.frame_id = "world"
        
        self.vins_path_pub.publish(self.vins_path)
    # def vins_callback(self, data):
    #     if self.vins_ori_offset == 0 or self.vins_pos_offset == 0:
    #         return
    #     # publishes vins_global

    #     # (global_position) = (position_offset) + (local_position)*(orientation_offset)
    #     # (global_orientation) = (orientation_offset)*(local_offset)

    #     ## pose
    #     vins_pos = data.pose.pose.position
    #     vins_ori = [data.pose.pose.orientation.x,
    #                 data.pose.pose.orientation.y,
    #                 data.pose.pose.orientation.z,
    #                 data.pose.pose.orientation.w]

    #     vins_global_pose = PoseStamped()
    #     vins_global_pose.header.stamp = data.header.stamp
    #     vins_global_pose.header.frame_id = "world"

    #     pos_q = [vins_pos.x, vins_pos.y, vins_pos.z, 0]
    #     rotated_pos = q_mul(q_mul(q_inv(self.vins_ori_offset), pos_q), self.vins_ori_offset)

    #     vins_global_pose.pose.position.x = self.vins_pos_offset[0] + rotated_pos[0]
    #     vins_global_pose.pose.position.y = self.vins_pos_offset[1] + rotated_pos[1]
    #     vins_global_pose.pose.position.z = self.vins_pos_offset[2] + rotated_pos[2]

    #     # [0]:x, [1]:y, [2]:z, [3]:w
    #     rotated_ori = q_mul(self.vins_ori_offset, vins_ori)
    #     vins_global_pose.pose.orientation.x = rotated_ori[0]
    #     vins_global_pose.pose.orientation.y = rotated_ori[1]
    #     vins_global_pose.pose.orientation.z = rotated_ori[2]
    #     vins_global_pose.pose.orientation.w = rotated_ori[3]

    #     self.vins_pose_pub.publish(vins_global_pose)


    #     ## odometry

    #     vins_global_odom = Odometry()
    #     vins_global_odom.header.stamp = data.header.stamp
    #     vins_global_odom.header.frame_id = "world"
    #     vins_global_odom.child_frame_id = "body"

    #     vins_global_odom.pose.pose.position.x = self.vins_pos_offset[0] + rotated_pos[0]
    #     vins_global_odom.pose.pose.position.y = self.vins_pos_offset[1] + rotated_pos[1]
    #     vins_global_odom.pose.pose.position.z = self.vins_pos_offset[2] + rotated_pos[2]
    #     vins_global_odom.pose.pose.orientation.x = rotated_ori[0]
    #     vins_global_odom.pose.pose.orientation.y = rotated_ori[1]
    #     vins_global_odom.pose.pose.orientation.z = rotated_ori[2]
    #     vins_global_odom.pose.pose.orientation.w = rotated_ori[3]

    #     self.vins_odom_pub.publish(vins_global_odom)


    #     ## path

    #     self.vins_path.poses.append(vins_global_pose)
    #     self.vins_path.header.stamp = rospy.Time.now()
    #     self.vins_path.header.frame_id = "world"
        
    #     self.vins_path_pub.publish(self.vins_path)


    #     ## modified pose
    #     vinsmodi_ori = rotated_ori

    #     vinsmodi_pose = PoseStamped()
    #     vinsmodi_pose.header.stamp = data.header.stamp
    #     vinsmodi_pose.header.frame_id = "world"

    #     modi_pos_q = [self.vins_pos_offset[0] + rotated_pos[0],
    #             self.vins_pos_offset[1] + rotated_pos[1],
    #             self.vins_pos_offset[2] + rotated_pos[2], 0]
    #     modi_rotated_pos = q_mul(q_mul(q_inv(self.world_ori_offset), modi_pos_q), self.world_ori_offset)
    #     vinsmodi_pose.pose.position.x = self.world_pos_offset[0] + modi_rotated_pos[0]
    #     vinsmodi_pose.pose.position.y = self.world_pos_offset[1] + modi_rotated_pos[1]
    #     vinsmodi_pose.pose.position.z = self.world_pos_offset[1] + modi_rotated_pos[2]
        
    #     modi_rotated_ori = q_mul(self.world_ori_offset, vinsmodi_ori)
    #     vinsmodi_pose.pose.orientation.x = modi_rotated_ori[0]
    #     vinsmodi_pose.pose.orientation.y = modi_rotated_ori[1]
    #     vinsmodi_pose.pose.orientation.z = modi_rotated_ori[2]
    #     vinsmodi_pose.pose.orientation.w = modi_rotated_ori[3]

    #     self.vinsmodi_pose_pub.publish(vinsmodi_pose)

    #     ## modified orientation
    #     vinsmodi_odom = Odometry()
    #     vinsmodi_odom.header.stamp = data.header.stamp
    #     vinsmodi_odom.header.frame_id = "world"
    #     vinsmodi_odom.child_frame_id = "body"

    #     vinsmodi_odom.pose.pose.position.x = self.world_pos_offset[0] + modi_rotated_pos[0]
    #     vinsmodi_odom.pose.pose.position.y = self.world_pos_offset[1] + modi_rotated_pos[1]
    #     vinsmodi_odom.pose.pose.position.z = self.world_pos_offset[2] + modi_rotated_pos[2]
    #     vinsmodi_odom.pose.pose.orientation.x = modi_rotated_ori[0]
    #     vinsmodi_odom.pose.pose.orientation.y = modi_rotated_ori[1]
    #     vinsmodi_odom.pose.pose.orientation.z = modi_rotated_ori[2]
    #     vinsmodi_odom.pose.pose.orientation.w = modi_rotated_ori[3]

    #     self.vinsmodi_odom_pub.publish(vinsmodi_odom)

    #     ## modified path

    #     self.vinsmodi_path.poses.append(vinsmodi_pose)
    #     self.vinsmodi_path.header.stamp = rospy.Time.now()
    #     self.vinsmodi_path.header.frame_id = "world"
        
    #     self.vinsmodi_path_pub.publish(self.vinsmodi_path)





if __name__ == '__main__':
    # define drones here
    if len(sys.argv) != 8:
        print("Usage: gps_with_offset.py x y z roll pitch yaw exp#")
        exit(0)
    rospy.init_node("vins_gps_with_offset", anonymous=True)
    xyz = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
    rpy = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]
    if sys.argv[7] == "1":
        d1 = Drone("UAV1", xyz, rpy, 1)
        d2 = Drone("UAV2", xyz, rpy, 1)
        d3 = Drone("UAV3", xyz, rpy, 1)
        d4 = Drone("UAV5", xyz, rpy, 1)
        d5 = Drone("UGV1", xyz, rpy, 1)

        # d4 = Drone("UAV4", xyz, rpy)
        # d5 = Drone("UAV5", xyz, rpy)
        rospy.spin()

    elif sys.argv[7] == "2":
        d1 = Drone("UAV1", xyz, rpy, 2)   
        d2 = Drone("UAV2", xyz, rpy, 2)
        d3 = Drone("UAV3", xyz, rpy, 2)
        d4 = Drone("UAV4", xyz, rpy, 2)
        d5 = Drone("UAV5", xyz, rpy, 2)

        rospy.spin()