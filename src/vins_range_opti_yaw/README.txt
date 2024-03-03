VIO optimized with UWB range measurements
(Beomjoon Park)
Reference : "UWB-VIO Fusiton for Accurate and Robust Relative Localization of Round Robotic Teams"


For usage:

rosrun vins_to_gps vins_to_gps
rosrun vins_range_opti_yaw vins_range_opti_yaw
rosbag play ~~~ --start=70 --duration=247

To use GPS data instead of VIO data for some drones, change the parameter of drones
defined in "main" of "vins_optimize_yaw.cpp"


Properties:
Uses sliding window optimization, Max freq = 20Hz
Discards roll and pitch (only uses xyz and yaw)
Multi-threaded, to simulate the decentralization of each drone
(optimization happens per drone)


Optimization weight settings (defined from line 110 of "vins_optimize_yaw.cpp"):

UWB stdvar : 1 m 
VIO covar : xyz = 0.01 m, yaw = 0.05 deg
prior covar : currently, shares the VIO covar 
            (should be modified as covariance carried over from historical measurments)


Problems: 
1. UWB range data are not filtered, since the experiment did not suffer from NLOS conditions.  
2. The covariance for residual of prior item is not properly implemented. 
3. The implementation discards roll and pitch. 
4. Initialization is done with ground truth (GPS data). 






hmm...

- I'm not sure if the optimization is running properly. 
The results are not significant. 

- I thought there were some problems when I use UAV5, but I 
get the same results even though I retry it with ground truth. 

- UWB filtering is not done. Didn't think it was necessary. 

- 