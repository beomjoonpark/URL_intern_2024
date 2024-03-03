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
from geometry_msgs.msg import PoseStamped,Point
from visualization_msgs.msg import Marker

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

color=[[255.0/255.0,255.0/255.0,255.0/255.0],
       [25.0/255.0,255.0/255.0,0.0/255.0],
       [32.0/255.0,74.0/255.0,135.0/255.0],
       [245.0/255.0,121.0,0.0/255.0],
       [237.0/255.0,212.0/255.0,0.0/255.0],
       [239.0/255.0,41.0/255.0,41.0/255.0],
       [0.0/255.0,255.0/255.0,255.0/255.0],
       [255.0/255.0,20.0/255.0,147.0/255.0]] #White,Green,Blue,Orange,Yellow,Red,Cyan,Pink

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
        self.out_marker_topic_name1 = rospy.get_param("~out_marker_topic_name1",'/marker1')
        self.color_picker= rospy.get_param("~color_pick","/color_pick")
        self.line_width= rospy.get_param("~line_width","/line_width")
        self.z_offset= rospy.get_param("~z_offset","/z_offset")

        rospy.Subscriber(self.input_topic_name1, PoseStamped, self.poseStamped_cb1)
        self.odom_pub1 = rospy.Publisher(self.out_pose_topic_name1, Odometry, queue_size=100)
        self.path_pub1 = rospy.Publisher(self.out_path_topic_name1, Path, queue_size=100)
	self.marker_pub1 = rospy.Publisher(self.out_marker_topic_name1, Marker, queue_size=100)
        self.path1 = Path()
	self.marker1 = Marker()
        self.marker1.action = Marker.ADD
	self.marker1.id=0
        self.marker1.type=Marker.LINE_STRIP
        self.marker1.pose.orientation.w=1.0
        self.marker1.color.r=color[self.color_picker][0]
        self.marker1.color.g=color[self.color_picker][1]
        self.marker1.color.b=color[self.color_picker][2]
        self.marker1.color.a=1.0
        self.marker1.scale.x=self.line_width
        self.offset1=[]

        self.rate = rospy.Rate(5)


    def poseStamped_cb1(self, msg):
        odometry = Odometry()
        odometry.header.stamp = msg.header.stamp
        odometry.header.frame_id = 'world'
        odometry.pose.pose.position = msg.pose.position
        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0
        odometry.pose.pose.orientation.z = 0.0
        odometry.pose.pose.orientation.w = 1.0
        if self.enable_file_write:
            self.f1 = open(self.file_folder+self.out_path_topic_name1+'.csv', "a")
            self.f1.write("%.6f, %.6f, %.6f, %.6f\n"%( (msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0) , odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z))
            self.f1.close()

        self.odom_pub1.publish(odometry)
        self.path1.poses.append(msg)
        self.path1.header.stamp = rospy.Time.now()
        self.path1.header.frame_id = self.parent_frame_id
        self.path_pub1.publish(self.path1)

        self.marker1.header.stamp = rospy.Time.now()
        self.marker1.header.frame_id = self.parent_frame_id

        point = Point()
        point.x=msg.pose.position.x
        point.y=msg.pose.position.y
        point.z=msg.pose.position.z+self.z_offset
        self.marker1.points.append(point)
        self.marker_pub1.publish(self.marker1)
        

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
