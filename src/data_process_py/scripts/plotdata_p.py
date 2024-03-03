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


def compare(d1, d2):
    r_list = []
    d_list = []
    t_list = []

    for gps_data1 in d1.gps_data:

        time = gps_data1[0]

        r = -1

        
        for range_data1 in d1.range_data:
            if range_data1[0] >= time:
                if range_data1[0]-0.1 >= time:
                    continue
                ind = indexofnode(range_data1[1], d2)
                if ind != -1:
                    r = range_data1[1][ind].dis
                    break
        # if r == -1:
        #     for range_data2 in d2.range_data:
        #         if range_data2[0] >= time:
        #             if range_data1[0]-0.1 >= time:
        #                 continue
        #             ind = indexofnode(range_data2[1], d1)
        #             if ind != -1:
        #                 r = range_data2[1][ind].dis
        if r == -1:
            continue
        
        d = -1
        for i in range(len(d2.gps_data)):
            if i ==0: continue

            gps_data2 = d2.gps_data[i]
            
            if gps_data2[0] > time:
                if gps_data2[0] -0.5 > time:
                    break
                if d2.gps_data[i-1][0] + 0.5 < time:
                    continue
                x2 = ((time-d2.gps_data[i-1][0])*d2.gps_data[i][1][0] + (d2.gps_data[i][0]-time)*d2.gps_data[i-1][1][0])/ (d2.gps_data[i][0] - d2.gps_data[i-1][0])
                y2 = ((time-d2.gps_data[i-1][0])*d2.gps_data[i][1][1] + (d2.gps_data[i][0]-time)*d2.gps_data[i-1][1][1])/ (d2.gps_data[i][0] - d2.gps_data[i-1][0])
                z2 = ((time-d2.gps_data[i-1][0])*d2.gps_data[i][1][2] + (d2.gps_data[i][0]-time)*d2.gps_data[i-1][1][2])/ (d2.gps_data[i][0] - d2.gps_data[i-1][0])
                
                x1 = gps_data1[1][0]
                y1 = gps_data1[1][1]
                z1 = gps_data1[1][2]

                d = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**0.5
                break

        if d == -1:
            continue

        r_list.append(r)
        d_list.append(d)
        t_list.append(time)
    


    return r_list, d_list, t_list






def plot_time_diff(d):
    time_diff = []
    for i in range(len(d.range_data)-1):
        time_diff.append(d.range_data[i+1][0] - d.range_data[i][0])
    plt.plot(time_diff)
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 3 and not (len(sys.argv) == 2 and sys.argv[1] == 'all'):
        print('Usage: "plotdata.py all" or "plotdata.py name1 name2"')
        exit(-1)
    if len(sys.argv) == 3 and sys.argv[1] not in NAME_TO_ID.keys():
        print('Invalid drone name: "'+sys.argv[1]+'"')
        exit(-1)
    if len(sys.argv) == 3 and sys.argv[2] not in NAME_TO_ID.keys():
        print('Invalid drone name: "'+sys.argv[2]+'"')
        exit(-1)
    
    if len(sys.argv) == 2:
        print("Compare all...")
        rospy.init_node("plotdata", anonymous=True)
        d1 = Drone("UAV1")
        d2 = Drone("UAV2")
        d3 = Drone("UAV3")
        d4 = Drone("UAV5")
        d5 = Drone("UGV1")
        print("Drone subscribers initialized.")
        rospy.spin()

        drone_list = [d1, d2, d3, d4, d5]


        for i in range(len(drone_list)):
            fig, ax = plt.subplots(3, 4, figsize=(16, 10))
            fig.suptitle(drone_list[i].name)
            # fig.suptitle("p_"+drone_list[i].name)

            cnt = 0
            for j in range(len(drone_list)):
                if i==j:
                    continue
                rl, dl, tl = compare(drone_list[i], drone_list[j])
                ax[0, cnt].plot(tl, rl, label='range')
                ax[0, cnt].plot(tl, dl, label='gps')

                # diff = [rl[k]-dl[k] for k in range(len(rl))]
                diff_p = [abs(rl[k]-dl[k])/dl[k]*100 for k in range(len(rl))]
                # ax[1, cnt].scatter(tl, diff, s=1)
                ax[1, cnt].scatter(tl, diff_p, s=1)
                
                # mean = np.mean(diff)
                # std = np.std(diff)
                # rmse = np.sqrt(np.mean(np.array(diff)**2))
                mean = np.mean(diff_p)
                std = np.std(diff_p)
                rmse = np.sqrt(np.mean(np.array(diff_p)**2))
                # ax[2, cnt].hist(diff, bins=40)
                ax[2, cnt].hist(diff_p, bins=40)


                ax[0, cnt].set_title("Distance between "+drone_list[i].name+" and "+drone_list[j].name)
                ax[0, cnt].legend()
                # ax[1, cnt].set_title("Range data - GPS distance")
                ax[1, cnt].set_title("Percentage Error")
                ax[2, cnt].set_title("Mean: "+str(round(mean, 6)) + ", std: "+str(round(std, 6))+ ", rmse: "+str(round(rmse, 6)), fontdict = {'fontsize' : 10})


                ax[0, cnt].set_xlim([50, 350])
                ax[0, cnt].set_ylim([0, 60])
                ax[1, cnt].set_xlim([50, 350])
                # ax[1, cnt].set_ylim([-27, 5])
                ax[1, cnt].set_ylim([0, 100])

                # ax[0, cnt].set(xlabel="Bag file time (s)", ylabel="Distance (m)")
                # ax[1, cnt].set(xlabel="Bag file time (s)", ylabel="Difference (m)")
                # ax[2, cnt].set(xlabel="Difference (m)", ylabel="Num of data")

                ax[0, cnt].set(xlabel="Bag file time (s)", ylabel="Distance (m)")
                ax[1, cnt].set(xlabel="Bag file time (s)", ylabel="Error (%)")
                ax[2, cnt].set(xlabel="Error (%)", ylabel="Num of data")

                cnt = cnt+1

            plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.90, wspace=0.35, hspace=0.4)

        
        plt.show()

        exit(0)


    print("Compare "+sys.argv[1]+" and "+sys.argv[2]+"...")
    rospy.init_node("plotdata", anonymous=True)
    d1 = Drone(sys.argv[1])
    d2 = Drone(sys.argv[2])
    print("Drone subscribers initialized.")
    rospy.spin()

    rl, dl, tl = compare(d1, d2)
    fig, ax = plt.subplots(3, 1, figsize=(4, 10))
    ax[0].plot(tl, rl, label='range')
    ax[0].plot(tl, dl, label='gps')
    ax[0].set_title(sys.argv[1]+" and "+sys.argv[2])
    ax[0].legend()

    diff = [rl[i]-dl[i] for i in range(len(rl))]
    ax[1].scatter(tl, diff, s=1)
    
    mean = np.mean(diff)
    std = np.std(diff)
    rmse = np.sqrt(np.mean(np.array(diff)**2))


    ax[2].hist(diff, bins=40)
    ax[2].set_title("Mean: "  +str(round(mean, 6)) + ", std : " + str(round(std, 6))+", rmse: "+str(round(rmse, 6)), fontdict = {'fontsize' : 10})


    plt.show()
    
    exit(0)