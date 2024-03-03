#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np
import utm

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class path_pub():
    def __init__(self):
        rospy.init_node('path_pubb', anonymous=True)
        self.parent_frame_id = rospy.get_param("~parent_frame_id", 'map')
        self.enable_altitude = rospy.get_param("/enable_altitude", False)
        self.enable_file_write = rospy.get_param("/enable_file_write", False)
        self.file_folder = rospy.get_param("/file_folder", "")
        
        self.input_topic_name1 = rospy.get_param("~input_topic_name1", '/fix')
        self.out_pose_topic_name1 = rospy.get_param("~out_pose_topic_name1", '/pose1')
        self.out_path_topic_name1 = rospy.get_param("~out_path_topic_name1", '/path1')


        rospy.Subscriber(self.input_topic_name1, Odometry, self.odom_cb1)
        self.pose_pub1 = rospy.Publisher(self.out_pose_topic_name1, PoseStamped, queue_size=100)
        self.path_pub1 = rospy.Publisher(self.out_path_topic_name1, Path, queue_size=100)
        self.path1 = Path()
        self.offset1=[]

        self.rate = rospy.Rate(5)


    def odom_cb1(self, msg):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'world'
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        if self.enable_file_write:
            self.f1 = open(self.file_folder+self.out_path_topic_name1+'.csv', "a")
            self.f1.write("%.6f, %.6f, %.6f, %.6f\n"%( (msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0) , pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
            self.f1.close()

        self.pose_pub1.publish(pose)
        self.path1.poses.append(pose)
        self.path1.header.stamp = rospy.Time.now()
        self.path1.header.frame_id = self.parent_frame_id
        self.path_pub1.publish(self.path1)

''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            path_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
