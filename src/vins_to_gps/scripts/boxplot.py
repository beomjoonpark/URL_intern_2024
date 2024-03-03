#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Times New Roman'



def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)
    plt.setp(bp['fliers'], markeredgecolor=color)



def boxplot(name, data, method_name, drone_name, color_list):


    plt.figure()

    n = len(method_name)
    offset = 3.2/(n-1)
    
    plotlist = []
    for i in range(len(method_name)):
        p = plt.boxplot(data[i], positions=np.array(xrange(len(data[i])))*6.0-1.6+i*offset,
                         sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
        set_box_color(p, color_list[i])
        plt.plot([], c=color_list[i], label=method_name[i])
        
        plotlist.append(p)
    plt.legend(loc='upper right')
    plt.xticks(xrange(0, len(drone_name)*6, 6), drone_name)
    plt.xlim(-3, (len(drone)-1)*6+3)
    plt.title(name)
    plt.xlabel("Robot")
    plt.ylabel("APE rmse (m)")
    plt.show()




    # p1 = plt.boxplot(data_a, positions=np.array(xrange(len(data_a)))*6.0-1.6, sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
    # p2 = plt.boxplot(data_b, positions=np.array(xrange(len(data_b)))*6.0-0.8, sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
    # p3 = plt.boxplot(data_c, positions=np.array(xrange(len(data_c)))*6.0, sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
    # p4 = plt.boxplot(data_d, positions=np.array(xrange(len(data_d)))*6.0+0.8, sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
    # p5 = plt.boxplot(data_e, positions=np.array(xrange(len(data_e)))*6.0+1.6, sym=None, widths=0.4, whiskerprops=dict(linestyle='--'))
    
    
    # set_box_color(p1, color_list[0]) 
    # set_box_color(p2, color_list[1])
    # set_box_color(p3, color_list[2])
    # set_box_color(p4, color_list[3])
    # set_box_color(p5, color_list[4])

    
    # plt.plot([], c=color_list[0], label=drone_name[0])
    # plt.plot([], c=color_list[1], label=drone_name[1])
    # plt.plot([], c=color_list[2], label=drone_name[2])
    # plt.plot([], c=color_list[3], label=drone_name[3])
    # plt.plot([], c=color_list[4], label=drone_name[4])

    # plt.legend(loc='upper right')

    # plt.xticks(xrange(0, len(method_name)*6, 6), method_name)
    # plt.xlim(-3, (len(method_name)-1)*6+3)
    # plt.title(name)
    # plt.show()



if __name__ == '__main__':
    ##### plot 1
    method1_data = [[1,2,5], [5,7,2,2,5], [7,2,5], [5,7,2,2,5], [7,2,5]]
    method2_data = [[6,4,2], [1,2,5,3,2], [2,3,5,1], [5,7,2,2,5], [7,2,5]]
    method3_data = [[1,2,5], [5,7,2,2,5], [7,2,5], [5,7,2,2,5], [7,2,5]]
    method4_data = [[6,4,2], [1,2,5,3,2], [2,3,5,1], [5,7,2,2,5], [7,2,5]]
    
    exp_name = "exp1"

    drone= ['UAV1', 'UAV2', 'UAV3', 'UAV5', 'UGV1']

    data = [method1_data, method2_data, method3_data, method4_data]
    method= ['A', 'B', 'C', 'D']
    color= ['red', 'blue', 'yellow', 'green']


    boxplot(exp_name, data, method, drone, color)



    method1_data = [[1,2,5], [5,7,2,2,5], [7,2,5], [5,7,2,2,5], [5,7,2,2,5]]
    method2_data = [[6,4,2], [1,2,5,3,2], [2,3,5,1], [5,7,2,2], [5,7,2,2,5]]
    method3_data = [[1,2,5], [5,7,2,2,5], [7,2,5], [5,7,2,2,5], [5,7,2,2,5]]
    method4_data = [[6,4,2], [1,2,5,3,2], [2,3,5,1], [5,7,2,2,5], [5,7,2,2,5]]
    method5_data = [[6,4,2], [1,2,5,3,2], [2,3,5,1], [5,7,2,2,5], [5,7,2,2,5]]

    exp_name = "exp2"

    drone= ['UAV1', 'UAV2', 'UAV3', 'UAV4', 'UAV5']

    data = [method1_data, method2_data, method3_data, method4_data, method5_data]
    method= ['A', 'B', 'C', 'D', 'E']
    color= ['red', 'blue', 'yellow', 'green', 'orange']


    boxplot(exp_name, data, method, drone, color)