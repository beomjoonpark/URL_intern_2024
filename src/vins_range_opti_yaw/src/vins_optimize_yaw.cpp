#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Range.h"
#include <math.h>
#include <iostream>
#include <chrono>
#include <vector>
#include "nlink_parser/LinktrackNodeframe2.h"
#include "nlink_parser/LinktrackNode2.h"
// #include "range_pub.h"
// #include "real_structure.h"
#include <thread>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/autodiff_cost_function.h"
// #include "ceres_header.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>
#include <deque>
#include <array>

#include "mutex"

#include "structure.h"
#include "cost.h"



using namespace std;



// pose inits for (exp1)
Eigen::Vector3d v1(0.046084799359169454, 0.08814165541493482, -0.18281563765009307);
Eigen::Vector3d v2(-0.7667722672943468, -27.861738568139515, -0.7427878895778954);
Eigen::Vector3d v3(-13.825226867023444, -7.744971852548965, -0.3947272812807685);
Eigen::Vector3d v4(12.774582405255853, -26.18224498094397, -0.037280355107611976);
Eigen::Vector3d v5(-11.31799676345569, -30.47547770819822, -0.1746036948719473);
Eigen::Quaterniond q1(0.9701525270324546, 0.001003165050067636, 0.006402456515928509, 0.24240930973157687);
Eigen::Quaterniond q2(0.9570360717412869, -0.007943698267386536, 0.018527014824972144, 0.2892676470522098);
Eigen::Quaterniond q3(0.9306886930615342, 0.0011263731185964748, 0.020406538886927904, 0.36524075741154377);
Eigen::Quaterniond q4(-0.953868818899681, 0.038640643826414536, -0.11493095991518963, -0.2746489730850827);
Eigen::Quaterniond q5(-0.9006272740107879, 0.00900127726727511, 0.001976556676219243, 0.43449457036321576);

// pose inits for (exp1, with time +17.5)
Eigen::Vector3d v1p(-0.0560235651409, 0.132127190676, 6.29819761563e-05);
Eigen::Quaterniond q1p(0.587048928033, -0.0291067839311, 0.0097672756158, 0.808969067116);
Eigen::Vector3d v2p(27.974902525, -0.0882004281685, 0.0290714094429);
Eigen::Quaterniond q2p(0.413551825949, -0.00575186623829, -0.034202364887, 0.909819763209);
Eigen::Vector3d v3p(8.56968444931, -13.3406288718, 0.0499946970182);
Eigen::Quaterniond q3p(0.293924503071, -0.00110865045294, 0.000384589097928, 0.955827918341);
Eigen::Vector3d v4p(24.8590531991, 13.6775037494, 0.0285498683626);
Eigen::Quaterniond q4p(0.900336279744, -0.0171153460221, -0.0137156303444, 0.43464184082);
Eigen::Vector3d v5p(31.0402606661, -9.21955539904, 0.0218965966532);
Eigen::Quaterniond q5p(0.90604970833, -0.0155986942046, 0.0170028846054, 0.422541724199);


double yaw1 = atan2(2*(q1.w()*q1.z()+q1.x()*q1.y()), q1.w()*q1.w()+q1.x()*q1.x()-q1.y()*q1.y()-q1.z()*q1.z());
double yaw2 = atan2(2*(q2.w()*q2.z()+q2.x()*q2.y()), q2.w()*q2.w()+q2.x()*q2.x()-q2.y()*q2.y()-q2.z()*q2.z());
double yaw3 = atan2(2*(q3.w()*q3.z()+q3.x()*q3.y()), q3.w()*q3.w()+q3.x()*q3.x()-q3.y()*q3.y()-q3.z()*q3.z());
double yaw4 = atan2(2*(q4.w()*q4.z()+q4.x()*q4.y()), q4.w()*q4.w()+q4.x()*q4.x()-q4.y()*q4.y()-q4.z()*q4.z());
double yaw5 = atan2(2*(q5.w()*q5.z()+q5.x()*q5.y()), q5.w()*q5.w()+q5.x()*q5.x()-q5.y()*q5.y()-q5.z()*q5.z());

double yaw1p = atan2(2*(q1p.w()*q1p.z()+q1p.x()*q1p.y()), q1p.w()*q1p.w()+q1p.x()*q1p.x()-q1p.y()*q1p.y()-q1p.z()*q1p.z());
double yaw2p = atan2(2*(q2p.w()*q2p.z()+q2p.x()*q2p.y()), q2p.w()*q2p.w()+q2p.x()*q2p.x()-q2p.y()*q2p.y()-q2p.z()*q2p.z());
double yaw3p = atan2(2*(q3p.w()*q3p.z()+q3p.x()*q3p.y()), q3p.w()*q3p.w()+q3p.x()*q3p.x()-q3p.y()*q3p.y()-q3p.z()*q3p.z());
double yaw4p = atan2(2*(q4p.w()*q4p.z()+q4p.x()*q4p.y()), q4p.w()*q4p.w()+q4p.x()*q4p.x()-q4p.y()*q4p.y()-q4p.z()*q4p.z());
double yaw5p = atan2(2*(q5p.w()*q5p.z()+q5p.x()*q5p.y()), q5p.w()*q5p.w()+q5p.x()*q5p.x()-q5p.y()*q5p.y()-q5p.z()*q5p.z());


Pose pose1(v1, 0, 0, yaw1);
Pose pose2(v2, 0, 0, yaw2);
Pose pose3(v3, 0, 0, yaw3);
Pose pose4(v4, 0, 0, yaw4);
Pose pose5(v5, 0, 0, yaw5);

Pose pose1p(v1p, 0, 0, yaw1p);
Pose pose2p(v2p, 0, 0, yaw2p);
Pose pose3p(v3p, 0, 0, yaw3p);
Pose pose4p(v4p, 0, 0, yaw4p);
Pose pose5p(v5p, 0, 0, yaw5p);




class Optimizer{
    public:

    int drone_num;
    array<Drone*, 5> dronelist;


    Optimizer(array<Drone*, 5> dl) {
        dronelist = dl;
        drone_num = dl.size();
    }

    void optimize(Drone* d0){
        // std::chrono::milliseconds dura(50);
        ros::Rate r(20);


        // Covariance settings. 

        double hist[5][4] = {{pow(0.01, 2), pow(0.01, 2), pow(0.01, 2), pow(0.05*acos(-1)/180, 2)},
                            {pow(0.01, 2), pow(0.01, 2), pow(0.01, 2), pow(0.05*acos(-1)/180, 2)},
                            {pow(0.01, 2), pow(0.01, 2), pow(0.01, 2), pow(0.05*acos(-1)/180, 2)},
                            {pow(0.01, 2), pow(0.01, 2), pow(0.01, 2), pow(0.05*acos(-1)/180, 2)},
                            {pow(0.01, 2), pow(0.01, 2), pow(0.01, 2), pow(0.05*acos(-1)/180, 2)}};
        int cnt = 1;
        double range_stdev = 0.2;

        Eigen::Matrix<double, 4, 4> vins_covar;
        vins_covar << pow(0.01, 2), 0, 0, 0,
                    0, pow(0.01, 2), 0, 0, 
                    0, 0, pow(0.01, 2), 0,
                    0, 0, 0, pow(0.1*acos(-1)/180, 2);



        while(ros::ok()){
            // lock all drone vins lock, d0 range lock
            Window* w = d0->window;

            // obtain lock
            for(int i = 0; i<drone_num; i++){
                dronelist[i]->vins_lock.lock();
            }
            d0->range_lock.lock();


            // check data
            int data_check = 1;
            for(int i = 0; i<drone_num; i++){
                if(i != d0->idx && d0->range_buff[i] == 0){
                    data_check = 0;
                    break;
                }
                if(dronelist[i]->vins_buff.p.x() == 0 && dronelist[i]->vins_buff.p.y() == 0 && dronelist[i]->vins_buff.p.z() == 0){
                    data_check = 0;
                    break;
                }
            }
            if (data_check == 0){
                for(int i = 0; i<drone_num; i++){
                    dronelist[i]->vins_lock.unlock();
                }
                d0->range_lock.unlock();
                r.sleep();
                continue;
            }
            
            // push pose data to window
            for(int i = 0; i<drone_num; i++){
                Pose prev = d0->vins_buff_prev[i];
                Pose now = dronelist[i]->vins_buff;
                Pose diff = prev.inv().mul(now);
                
                if(w->pose[i].empty()) w->pose[i].push_back(w->m_pose[i].mul(diff));
                else w->pose[i].push_back(w->pose[i].back().mul(diff));

                w->pose_diff[i].push_back(diff);
                // update vins_buff_pref
                d0->vins_buff_prev[i] = now;
            }
            // push d0 range to window
            float dest[drone_num];
            std::memcpy(&dest, &d0->range_buff, sizeof(d0->range_buff));

            w->range_queue.push_back(dest);

            w->time_queue.push_back(d0->time);


            // release all lock
            for(int i = 0; i<drone_num; i++){
                dronelist[i]->vins_lock.unlock();
            }
            d0->range_lock.unlock();


            // check window size
            // if 20:
            //   do optimization with 20 and 1(marginalized) data using P, R, O cost
            //
            //   pop out extra marginalized data
            //   update window
            //
            // sleep with f Hz.
            if (w->size() == NUM_WINDOWS){
                ceres::Problem problem;
                ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
                ceres::LossFunction* huber = new ceres::HuberLoss(1);
                Eigen::Matrix<double, 4, 4> hist_covar;

                // P cost
                for(int i = 0; i<drone_num;i++){


                    /*
                     * The covariance for the P cost is not set properly. 
                     * Supposed to use "covariance carried out over from 
                     * historical measurements". 
                     * 
                    */
                    hist_covar << hist[i][0], 0, 0, 0,
                                    0, hist[i][1], 0, 0,
                                    0, 0, hist[i][2], 0,
                                    0, 0, 0, hist[i][3];

                    ceres::CostFunction* pcost = P_cost::Create(w->pose_diff[i][0], w->m_pose[i], vins_covar);
                    problem.AddResidualBlock(pcost, nullptr, w->pose[i][0].p.data(), &w->pose[i][0].yaw);
                }
                // R cost
                for(int i = 0; i<drone_num; i++){
                    if(d0->idx == i){ continue; }
                    for(int t = 0; t<NUM_WINDOWS; t++){
                        float r = w->range_queue[t][i];
                        ceres::CostFunction* rcost = R_cost::Create(r, range_stdev);
                        problem.AddResidualBlock(rcost, huber, w->pose[i][t].p.data(), w->pose[d0->idx][t].p.data());
                    }
                }
                // O cost
                for(int i = 0; i<drone_num; i++){
                    for(int t = 1; t<NUM_WINDOWS; t++){
                        ceres::CostFunction* ocost = O_cost::Create(w->pose_diff[i][t], vins_covar);
                        problem.AddResidualBlock(ocost, nullptr, w->pose[i][t-1].p.data(),
                                                                &w->pose[i][t-1].yaw,
                                                                w->pose[i][t].p.data(),
                                                                &w->pose[i][t].yaw);
                    }
                }

                ceres::Solver::Options options;
                // options.function_tolerance = 0;
                options.max_num_iterations = 50;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                std::cout << summary.FullReport() << endl;

                // update P cost covariance
                // for(int i = 0; i<drone_num;i++){
                //     Pose a = w->pose[i][0];
                //     Pose m = w->m_pose[i];
                //     Pose z = w->pose_diff[i][0];
                //     Pose expected_a = m.mul(z);


                //     hist[i][0] += pow(a.p.x()-expected_a.p.x(), 2);
                //     hist[i][1] += pow(a.p.y()-expected_a.p.y(), 2);
                //     hist[i][2] += pow(a.p.z()-expected_a.p.z(), 2);
                //     hist[i][3] += pow(a.yaw-expected_a.yaw, 2);               
                // }
                // cnt += 1;

                // pop marginalized data
                for(int i = 0; i<drone_num;i++){
                    w->m_pose[i] = w->pose[i].front();
                    w->pose[i].pop_front();
                    w->pose_diff[i].pop_front();
                }
                w->range_queue.pop_front();
                ros::Time t = w->time_queue.front();
                w->time_queue.pop_front();
                

                std::cout << "publishing" << std::endl;
                


                Pose pub = w->init.mul(w->m_pose[d0->idx]);



                Eigen::Quaterniond pubq = Eigen::AngleAxisd(pub.roll, Eigen::Vector3d::UnitX())
                                        * Eigen::AngleAxisd(pub.pitch, Eigen::Vector3d::UnitY())
                                        * Eigen::AngleAxisd(pub.yaw, Eigen::Vector3d::UnitZ());

                
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = t;
                pose.header.frame_id = "world";
                pose.pose.position.x = pub.p.x();
                pose.pose.position.y = pub.p.y();
                pose.pose.position.z = pub.p.z();
                pose.pose.orientation.x = pubq.x();
                pose.pose.orientation.y = pubq.y();
                pose.pose.orientation.z = pubq.z();
                pose.pose.orientation.w = pubq.w();
                d0->pose_pub.publish(pose);

                nav_msgs::Odometry odom;
                odom.header.stamp = t;
                odom.header.frame_id = "world";
                odom.child_frame_id = "body";
                odom.pose.pose.position.x = pub.p.x();
                odom.pose.pose.position.y = pub.p.y();
                odom.pose.pose.position.z = pub.p.z();
                odom.pose.pose.orientation.x = pubq.x();
                odom.pose.pose.orientation.y = pubq.y();
                odom.pose.pose.orientation.z = pubq.z();
                odom.pose.pose.orientation.w = pubq.w();
                d0->odom_pub.publish(odom);

                d0->path.header.stamp = t;
                d0->path.header.frame_id = "world";
                d0->path.poses.push_back(pose);
                d0->path_pub.publish(d0->path);
                
            }

            r.sleep();
        }
        std::cout << "dead" << std::endl;
   }
};




int main(int argc, char **argv){

    ros::init(argc, argv, "vins_optimize_yaw_node");

    Window* window1;
    Window* window2;
    Window* window3;
    Window* window4;
    Window* window5;
    if(strcmp(argv[1], "gps")==0){
        window1 = new Window(pose1, pose2, pose3, pose4, pose5, pose1);
        window2 = new Window(pose1, pose2, pose3, pose4, pose5, pose2);
        window3 = new Window(pose1, pose2, pose3, pose4, pose5, pose3);
        window4 = new Window(pose1, pose2, pose3, pose4, pose5, pose4);
        window5 = new Window(pose1, pose2, pose3, pose4, pose5, pose5);
    }
    else if(strcmp(argv[1], "given")==0){
        window1 = new Window(pose1p, pose2p, pose3p, pose4p, pose5p, pose1p);
        window2 = new Window(pose1p, pose2p, pose3p, pose4p, pose5p, pose2p);
        window3 = new Window(pose1p, pose2p, pose3p, pose4p, pose5p, pose3p);
        window4 = new Window(pose1p, pose2p, pose3p, pose4p, pose5p, pose4p);
        window5 = new Window(pose1p, pose2p, pose3p, pose4p, pose5p, pose5p);
    }
    else{
        std::cout << "gps or given"<< std::endl;
        return 0;
    }


    Drone* drone1 = new Drone("UAV1", 1, window1, false);
    Drone* drone2 = new Drone("UAV2", 2, window2, false);
    Drone* drone3 = new Drone("UAV3", 3, window3, false);
    
    Drone* drone4 = new Drone("UAV5", 5, window4, false);
    // Drone* drone4 = new Drone("UAV4", 4, window4, false);
    Drone* drone5 = new Drone("UGV1", 9, window5, false);
    // Drone* drone5 = new Drone("UAV5", 5, window5, false);
    
    
    array<Drone*, 5> dronelist = {drone1, drone2, drone3, drone4, drone5};
    // array<Drone*, 3> dronelist = {drone1, drone2, drone3};

    Optimizer optimizer(dronelist);

    thread t0(&Optimizer::optimize, &optimizer, optimizer.dronelist[0]);
    thread t1(&Optimizer::optimize, &optimizer, optimizer.dronelist[1]);
    thread t2(&Optimizer::optimize, &optimizer, optimizer.dronelist[2]);
    thread t3(&Optimizer::optimize, &optimizer, optimizer.dronelist[3]);
    thread t4(&Optimizer::optimize, &optimizer, optimizer.dronelist[4]);

    ros::spin();

    return 0;
}