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

#include "structure2.h"
#include "cost2.h"



using namespace std;

double q2y(Eigen::Quaterniond q){
    return atan2(2*(q.w()*q.z()+q.x()*q.y()),
                    q.w()*q.w()+q.x()*q.x()-q.y()*q.y()-q.z()*q.z());
}


// (exp2)
Eigen::Vector3d offset_v1(0.2839265424023872, 0.3501335000642113, -0.8898000354769099);
Eigen::Quaterniond offset_q1(0.9696955, 0, 0, 0.2443168);
Eigen::Vector3d offset_v2(3.6664188894947767, -5.8602600183193625, 0.6172419963716148);
Eigen::Quaterniond offset_q2(0.9469306, 0, 0, 0.3214381);
Eigen::Vector3d offset_v3(-2.035869980905222, 4.850016238263976, 0.15399615833327587);
Eigen::Quaterniond offset_q3(0.9572383, 0, 0, 0.2893005);
Eigen::Vector3d offset_v4(6.632136889366072, -9.921340631853768, 0.8212452898631724);
Eigen::Quaterniond offset_q4(0.9556141, 0, 0, 0.2946214);
Eigen::Vector3d offset_v5(-5.235829267022119, 10.441634395892562, 0.5710852273549998);
Eigen::Quaterniond offset_q5(0.944544, 0, 0, 0.3283839);
double offset_yaw1 = q2y(offset_q1);
double offset_yaw2 = q2y(offset_q2);
double offset_yaw3 = q2y(offset_q3);
double offset_yaw4 = q2y(offset_q4);
double offset_yaw5 = q2y(offset_q5);
Pose offset1(offset_v1, 0, 0, offset_yaw1);
Pose offset2(offset_v2, 0, 0, offset_yaw2);
Pose offset3(offset_v3, 0, 0, offset_yaw3);
Pose offset4(offset_v4, 0, 0, offset_yaw4);
Pose offset5(offset_v5, 0, 0, offset_yaw5);


Eigen::Vector3d local_v1(0.00019100995840280722, 0.00015999313526430267, -0.00019343218885974823);
Eigen::Quaterniond local_q1(0.9999999949853364, 1.1741043137926272e-05, -7.674855341963667e-05, 6.325452229816205e-05);
Eigen::Vector3d local_v2(0.00029686092259461195, 0.00011302363591749912, -0.00025256054801543764);
Eigen::Quaterniond local_q2(0.99999995190435, 0.00015548216935559666, -0.00025662693775561527, -7.848061907113819e-05);
Eigen::Vector3d local_v3(-9.97045917086852e-05, 0.00032904019174678203, 1.3197070020491505e-05);
Eigen::Quaterniond local_q3(0.9999999788763945, -0.0001388635296306502, -9.154474614424549e-05, -0.00012076295048373441);
Eigen::Vector3d local_v4(0.0027010140368008855, -0.00018905894927995806, -0.0002497448126700555);
Eigen::Quaterniond local_q4(0.9999999899874848, 7.330757775248406e-05, -8.356436610430452e-05, -8.756726564583662e-05);
Eigen::Vector3d local_v5(0.0007968799986347458, -1.1568430523318339e-05, 0.00090667276949037);
Eigen::Quaterniond local_q5(0.9999957297490483, 0.00031753176413394825, 0.0029048262166890874, 4.052034032438935e-05);
double local_yaw1 = q2y(local_q1);
double local_yaw2 = q2y(local_q2);
double local_yaw3 = q2y(local_q3);
double local_yaw4 = q2y(local_q4);
double local_yaw5 = q2y(local_q5);
Pose local1(local_v1, 0, 0, local_yaw1);
Pose local2(local_v2, 0, 0, local_yaw2);
Pose local3(local_v3, 0, 0, local_yaw3);
Pose local4(local_v4, 0, 0, local_yaw4);
Pose local5(local_v5, 0, 0, local_yaw5);

Pose pose1 = offset1.mul(local1);
Pose pose2 = offset2.mul(local2);
Pose pose3 = offset3.mul(local3);
Pose pose4 = offset4.mul(local4);
Pose pose5 = offset5.mul(local5);



// without initial pose (estimated pose)

Eigen::Vector3d v1p(-0.0123785124597, 0.00849374929637, 0.0386940839794);
Eigen::Quaterniond q1p(0.658871315646, -0.0564253819382, 0.0199359131306, 0.749871539039);
Eigen::Vector3d v2p(6.11638354535, 0.0712668810255, 0.00115638295049);
Eigen::Quaterniond q2p(0.732547158558, -0.0106439551517, -0.0108320768475, 0.680546863058);
Eigen::Vector3d v3p(-5.60222640304, -1.83490080803, -0.039282624083);
Eigen::Quaterniond q3p(0.67893518862, -0.0383182587986, -0.008059246473, 0.733153305416);
Eigen::Vector3d v4p(11.6412503051, 1.98684827932, 0.0892225850464);
Eigen::Quaterniond q4p(0.790634027917, -0.0304270381418, 0.0224104822926, 0.61112175508);
Eigen::Vector3d v5p(-11.828091146, -3.27977338912, -0.0931421601189);
Eigen::Quaterniond q5p(0.731359195992, -0.0162494787389, -0.0141017502589, 0.681653006681);
double yaw1p = q2y(q1p);
double yaw2p = q2y(q2p);
double yaw3p = q2y(q3p);
double yaw4p = q2y(q4p);
double yaw5p = q2y(q5p);

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

    std::cout<<pose1p.yaw<<std::endl;
    std::cout<<pose2p.yaw<<std::endl;
    std::cout<<pose3p.yaw<<std::endl;
    std::cout<<pose4p.yaw<<std::endl;
    std::cout<<pose5p.yaw<<std::endl;

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
    
    // Drone* drone4 = new Drone("UAV5", 5, window4, false);
    Drone* drone4 = new Drone("UAV4", 4, window4, false);
    // Drone* drone5 = new Drone("UGV1", 9, window5, false);
    Drone* drone5 = new Drone("UAV5", 5, window5, false);
    
    
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