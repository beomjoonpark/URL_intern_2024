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



//initial pose
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

Pose pose1(v1, q1);
Pose pose2(v2, q2);
Pose pose3(v3, q3);
Pose pose4(v4, q4);
Pose pose5(v5, q5);



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
        ros::Rate r(10);
        double hist[5][6] = {{10, 10, 1, 0.0001, 0.0001, 0.0001},
                            {10, 10, 1, 0.0001, 0.0001, 0.0001},
                            {10, 10, 1, 0.0001, 0.0001, 0.0001},
                            {10, 10, 1, 0.0001, 0.0001, 0.0001},
                            {10, 10, 1, 0.0001, 0.0001, 0.0001},};
        int cnt = 1;

        
        double range_stdev = 0.5;

        Eigen::Matrix<double, 6, 6> vins_covar;
        vins_covar << 13, 0, 0, 0, 0, 0,
                    0, 13, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0.0001, 0, 0,
                    0, 0, 0, 0, 0.0001, 0,
                    0, 0, 0, 0, 0, 0.0001;



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
                Eigen::Matrix<double, 6, 6> hist_covar;

                // P cost
                for(int i = 0; i<drone_num;i++){
                    ////////////////////
                    if(i == 3){continue;}
                    ////////////////////
                    hist_covar << hist[i][0]/cnt, 0, 0, 0 ,0 ,0,
                                    0, hist[i][1]/cnt, 0, 0, 0 ,0,
                                    0, 0, hist[i][2]/cnt, 0, 0, 0,
                                    0, 0, 0, hist[i][3]/cnt, 0, 0,
                                    0, 0, 0, 0, hist[i][4]/cnt, 0, 
                                    0, 0, 0, 0, 0, hist[i][5]/cnt;
                    // hist_covar << hist[i][0], 0, 0, 0 ,0 ,0,
                    //                 0, hist[i][1], 0, 0, 0 ,0,
                    //                 0, 0, hist[i][2], 0, 0, 0,
                    //                 0, 0, 0, hist[i][3], 0, 0,
                    //                 0, 0, 0, 0, hist[i][4], 0, 
                    //                 0, 0, 0, 0, 0, hist[i][5];

                    ceres::CostFunction* pcost = P_cost::Create(w->pose_diff[i][0], w->m_pose[i], hist_covar);
                    problem.AddResidualBlock(pcost, nullptr, w->pose[i][0].p.data(), w->pose[i][0].q.coeffs().data());
                    problem.SetManifold(w->pose[i][0].q.coeffs().data(), quaternion_manifold);
                }
                // R cost
                for(int i = 0; i<drone_num; i++){
                    if(d0->idx == i){ continue; }
                    ////////////////////
                    if(i == 3){continue;}
                    ////////////////////
                    for(int t = 0; t<NUM_WINDOWS; t++){
                        float r = w->range_queue[t][i];
                        ceres::CostFunction* rcost = R_cost::Create(r, range_stdev);
                        problem.AddResidualBlock(rcost, huber, w->pose[i][t].p.data(), w->pose[d0->idx][t].p.data());
                    }
                }
                // O cost
                for(int i = 0; i<drone_num; i++){
                    ////////////////////
                    if(i == 3){continue;}
                    ////////////////////

                    for(int t = 1; t<NUM_WINDOWS; t++){
                        ceres::CostFunction* ocost = O_cost::Create(w->pose_diff[i][t], vins_covar);
                        problem.AddResidualBlock(ocost, nullptr, w->pose[i][t-1].p.data(),
                                                                w->pose[i][t-1].q.coeffs().data(),
                                                                w->pose[i][t].p.data(),
                                                                w->pose[i][t].q.coeffs().data());
                        problem.SetManifold(w->pose[i][t-1].q.coeffs().data(), quaternion_manifold);
                        problem.SetManifold(w->pose[i][t].q.coeffs().data(), quaternion_manifold);
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
                for(int i = 0; i<drone_num;i++){
                    Pose a = w->pose[i][0];
                    Pose m = w->m_pose[i];
                    Pose z = w->pose_diff[i][0];
                    Eigen::Matrix<double, 3, 1> p_diff = (a.p - (z.q.conjugate() * a.q)*z.p) - m.p;
                    Eigen::Quaterniond q_diff = (z.q.conjugate()*a.q) * m.q.conjugate();
                    hist[i][0] += pow(p_diff.x(), 2);
                    hist[i][1] += pow(p_diff.y(), 2);
                    hist[i][2] += pow(p_diff.z(), 2);
                    hist[i][3] += pow(q_diff.x(), 2);
                    hist[i][4] += pow(q_diff.y(), 2);
                    hist[i][5] += pow(q_diff.z(), 2);
                }
                cnt = cnt + 1;


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

                
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = t;
                pose.header.frame_id = "world";
                pose.pose.position.x = pub.p.x();
                pose.pose.position.y = pub.p.y();
                pose.pose.position.z = pub.p.z();
                pose.pose.orientation.x = pub.q.x();
                pose.pose.orientation.y = pub.q.y();
                pose.pose.orientation.z = pub.q.z();
                pose.pose.orientation.w = pub.q.w();
                d0->pose_pub.publish(pose);

                nav_msgs::Odometry odom;
                odom.header.stamp = t;
                odom.header.frame_id = "world";
                odom.child_frame_id = "body";
                odom.pose.pose.position.x = pub.p.x();
                odom.pose.pose.position.y = pub.p.y();
                odom.pose.pose.position.z = pub.p.z();
                odom.pose.pose.orientation.x = pub.q.x();
                odom.pose.pose.orientation.y = pub.q.y();
                odom.pose.pose.orientation.z = pub.q.z();
                odom.pose.pose.orientation.w = pub.q.w();
                d0->odom_pub.publish(odom);

                d0->path.header.stamp = t;
                d0->path.header.frame_id = "world";
                d0->path.poses.push_back(pose);
                d0->path_pub.publish(d0->path);
                
                // geometry_msgs::PoseStamped pose;
                // pose.header.stamp = ros::Time::now();
                // pose.header.frame_id = "world";
                // pose.pose.position.x = w->m_pose[d0->idx].p.x();
                // pose.pose.position.y = w->m_pose[d0->idx].p.y();
                // pose.pose.position.z = w->m_pose[d0->idx].p.z();
                // pose.pose.orientation.x = w->m_pose[d0->idx].q.x();
                // pose.pose.orientation.y = w->m_pose[d0->idx].q.y();
                // pose.pose.orientation.z = w->m_pose[d0->idx].q.z();
                // pose.pose.orientation.w = w->m_pose[d0->idx].q.w();
                // d0->pose_pub.publish(pose);

                // nav_msgs::Odometry odom;
                // odom.header.stamp = ros::Time::now();
                // odom.header.frame_id = "world";
                // odom.child_frame_id = "body";
                // odom.pose.pose.position.x = w->m_pose[d0->idx].p.x();
                // odom.pose.pose.position.y = w->m_pose[d0->idx].p.y();
                // odom.pose.pose.position.z = w->m_pose[d0->idx].p.z();
                // odom.pose.pose.orientation.x = w->m_pose[d0->idx].q.x();
                // odom.pose.pose.orientation.y = w->m_pose[d0->idx].q.y();
                // odom.pose.pose.orientation.z = w->m_pose[d0->idx].q.z();
                // odom.pose.pose.orientation.w = w->m_pose[d0->idx].q.w();
                // d0->odom_pub.publish(odom);

                // d0->path.header.stamp = ros::Time::now();
                // d0->path.header.frame_id = "world";
                // d0->path.poses.push_back(pose);
                // d0->path_pub.publish(d0->path);
            }
            // std::this_thread::sleep_for(dura);
            r.sleep();
        }
        std::cout << "dead" << std::endl;
   }
};




int main(int argc, char **argv){

    ros::init(argc, argv, "vins_optimize_node");
    
    Window* window1 = new Window(pose1, pose2, pose3, pose4, pose5, pose1);
    Window* window2 = new Window(pose1, pose2, pose3, pose4, pose5, pose2);
    Window* window3 = new Window(pose1, pose2, pose3, pose4, pose5, pose3);
    Window* window4 = new Window(pose1, pose2, pose3, pose4, pose5, pose4);
    Window* window5 = new Window(pose1, pose2, pose3, pose4, pose5, pose5);

    Drone* drone1 = new Drone("UAV1", 1, window1);
    Drone* drone2 = new Drone("UAV2", 2, window2);
    Drone* drone3 = new Drone("UAV3", 3, window3);
    Drone* drone4 = new Drone("UAV5", 5, window4);
    Drone* drone5 = new Drone("UGV1", 9, window5);
    
    
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