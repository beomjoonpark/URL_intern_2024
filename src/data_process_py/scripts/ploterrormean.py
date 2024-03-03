#!/usr/bin/env python
from matplotlib import pyplot as plt
import rospy
import nlink_parser.msg
from geometry_msgs.msg import PoseStamped
import sys
import numpy as np


NAME_TO_ID = {
    "UAV1" : 1,
    "UAV2" : 2,
    "UAV3" : 3,
    "UAV5" : 5,
    "UGV1" : 9    
}

#ros_time = given time - offset
#abs_time = rostime - timestamp_offset
rostime_offset = 1701765389.227696896 - 65
time_offset = {     # global time + offset = time, offset = time - global time
    "UAV1_gps" : 1701765388.512625344-1701765389.289641857,
    "UAV1_range" : 12768.987-1701765389.295660496,

    "UAV2_gps" : 1701765388.609975160-1701765389.246841192,
    "UAV2_range" : 12768.466-1701765389.258947611,
    
    "UAV3_gps" : 1701765389.225029320-1701765389.228278875,
    "UAV3_range" : 12768.919-1701765389.247067451,

    "UAV5_gps" : 1701765387.732106512-1701765389.243580580,
    "UAV5_range" : 12767.481-1701765389.245367765,

    "UGV1_gps" : 1701765387.669084207-1701765389.367691755,
    "UGV1_range" : 12768.985-1701765389.377741814
}


class Drone:
    def __init__(self, name):
        self.gps_data = []
        self.range_data = []
        self.name = name
        self.gps_sub = rospy.Subscriber("/"+name+"_gps_pose", PoseStamped, self.gps_callback)
        self.range_sub = rospy.Subscriber("/"+name+"/nlink_linktrack_nodeframe2", nlink_parser.msg.LinktrackNodeframe2, self.range_callback)

    def gps_callback(self, data):
        header_time = data.header.stamp.secs+data.header.stamp.nsecs*10**(-9)
        ros_time = header_time - time_offset[self.name+"_gps"]
        abs_time =  ros_time - rostime_offset
        self.gps_data.append((abs_time, (data.pose.position.x, data.pose.position.y, data.pose.position.z)))

    def range_callback(self, data):
        # if not self.range_data:
        #     if data.system_time * 10**(-3) < self.range_data[-1][0]:
        #         return
        header_time = data.system_time*10**(-3)
        ros_time = header_time - time_offset[self.name+"_range"]
        abs_time = ros_time - rostime_offset
        self.range_data.append((abs_time, data.nodes))

def indexofnode(nodes, d):
    id = NAME_TO_ID[d.name]
    for i in range(len(nodes)):
        if nodes[i].id == id:
            return i
        
    return -1


def compare_all(drone_list):
    time_list = []
    range_list = []
    gps_dist_list = []

    for gps_data1 in drone_list[0].gps_data:
        time = gps_data1[0]
        r_list = [] #list of length 10
        for i in range(len(drone_list)-1):
            for j in range(i+1, len(drone_list)):
                r = -1

                d1 = drone_list[i]
                d2 = drone_list[j]

                for range_data1 in d1.range_data:
                    if range_data1[0] >= time:
                        if range_data1[0]-0.1 >= time:
                            continue
                        ind = indexofnode(range_data1[1], d2)
                        if ind != -1:
                            r = range_data1[1][ind].dis
                            r_list.append(r)
                            break
                if r == -1: break
            if r == -1: break
        if r == -1: continue

        position = []

        position.append((gps_data1[1][0], gps_data1[1][1], gps_data1[1][2]))

        for i in range(len(drone_list)):
            if i==0: continue
            d2 = drone_list[i]
            p = (0, 0, 0)
            for j in range(len(d2.gps_data)):  #for all gps data of d2
                if j ==0: continue
                gps_data2 = d2.gps_data[j]
                if gps_data2[0] > time:         # if it goes over some certain time
                    if gps_data2[0] -0.5 > time:    # if data is too far, no data
                        break
                    if d2.gps_data[j-1][0] + 0.5 < time:    #if prev data is too far, check if there are better data
                        continue
                    x2 = ((time-d2.gps_data[j-1][0])*d2.gps_data[j][1][0] + (d2.gps_data[j][0]-time)*d2.gps_data[j-1][1][0])/ (d2.gps_data[j][0] - d2.gps_data[j-1][0])
                    y2 = ((time-d2.gps_data[j-1][0])*d2.gps_data[j][1][1] + (d2.gps_data[j][0]-time)*d2.gps_data[j-1][1][1])/ (d2.gps_data[j][0] - d2.gps_data[j-1][0])
                    z2 = ((time-d2.gps_data[j-1][0])*d2.gps_data[j][1][2] + (d2.gps_data[j][0]-time)*d2.gps_data[j-1][1][2])/ (d2.gps_data[j][0] - d2.gps_data[j-1][0])
                    p = (x2, y2, z2)
                    position.append(p)
                    break
            if p == (0,0,0): break
        if p == (0,0,0): continue

        d_list = []
        for i in range(len(drone_list)-1):
            for j in range(i+1, len(drone_list)):
                d = ((position[i][0]-position[j][0])**2 + (position[i][1]-position[j][1])**2 + (position[i][2]-position[j][2])**2)**0.5
                d_list.append(d)
        gps_dist_list.append(d_list)
        time_list.append(time)
        range_list.append(r_list)
    


    return range_list, gps_dist_list, time_list






def plot_time_diff(d):
    time_diff = []
    for i in range(len(d.range_data)-1):
        time_diff.append(d.range_data[i+1][0] - d.range_data[i][0])
    plt.plot(time_diff)
    plt.show()


if __name__ == '__main__':

    
    rospy.init_node("ploterrormean", anonymous=True)
    d1 = Drone("UAV1")
    d2 = Drone("UAV2")
    d3 = Drone("UAV3")
    d4 = Drone("UAV5")
    d5 = Drone("UGV1")
    print("Drone subscribers initialized.")
    rospy.spin()

    drone_list = [d1, d2, d3, d4, d5]
    diff_list = []
    diff_p_list = []
    ind = []

    rl, dl, tl = compare_all(drone_list)

    print(len(tl))
    cnt = 0
    for i in range(len(drone_list)-1):
        for j in range(i+1, len(drone_list)):
            diff = [rl[k][cnt]-dl[k][cnt] for k in range(len(tl))]
            diff_p = [abs(rl[k][cnt]-dl[k][cnt])/dl[k][cnt]*100 for k in range(len(tl))]
            diff_list.append(diff)
            diff_p_list.append(diff_p)
            ind.append((i, j))

            cnt = cnt + 1
    
    diff_list_wo = [[],[],[],[],[],[],[],[],[],[]]
    diff_p_list_wo =[[],[],[],[],[],[],[],[],[],[]]
    tl_wo = []
    for i in range(len(tl)):
        if abs(diff_list[-1][i]) > 15:
            continue
        for j in range(10):
            diff_list_wo[j].append(diff_list[j][i])
            diff_p_list_wo[j].append(diff_p_list[j][i])
        tl_wo.append(tl[i])
        
        

    # fig1 : with error peak, fig2: without error peak, [0] : m, [1] : %
    fig1, ax1 = plt.subplots(1, 2, figsize=(15, 5))
    fig2, ax2 = plt.subplots(1, 2, figsize=(15, 5))


    ax1[0].plot(tl, np.mean(np.array(diff_list), axis=0), linewidth=0.5)
    ax1[1].plot(tl, np.mean(np.array(diff_p_list), axis=0), linewidth=0.5)
    ax2[0].plot(tl_wo, np.mean(np.array(diff_list_wo), axis=0), linewidth=0.5)
    ax2[1].plot(tl_wo, np.mean(np.array(diff_p_list_wo), axis=0), linewidth=0.5)
    

    fig1.suptitle("with peak error")
    fig2.suptitle("without peak error")

    ax1[0].set_xlim([50, 350])
    ax1[0].set_ylim([-27, 5])
    ax1[1].set_xlim([50, 350])
    ax1[1].set_ylim([0, 70])

    ax2[0].set_xlim([50, 350])
    ax2[0].set_ylim([-10, 5])
    ax2[1].set_xlim([50, 350])
    ax2[1].set_ylim([0, 60])

    ax1[0].set_title("Mean (Range data - GPS distance)")
    ax1[1].set_title("Mean (Percentage Error)")
    ax1[0].set(xlabel="Bag file time (s)", ylabel="Difference (m)")
    ax1[1].set(xlabel="Bag file time (s)", ylabel="Error (%)")

    ax2[0].set_title("Mean (Range data - GPS distance)")
    ax2[1].set_title("Mean (Percentage Error)")
    ax2[0].set(xlabel="Bag file time (s)", ylabel="Difference (m)")
    ax2[1].set(xlabel="Bag file time (s)", ylabel="Error (%)")


    ax1[0].legend(fontsize="6")
    ax1[1].legend(fontsize="6")
    ax2[0].legend(fontsize="6")
    ax2[1].legend(fontsize="6")

    plt.show()

    exit(0)

