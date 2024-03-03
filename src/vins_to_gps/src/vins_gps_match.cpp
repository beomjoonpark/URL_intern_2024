#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <iostream>
#include <chrono>
#include <vector>

#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>
#include <deque>
#include <array>
#include "mutex"
#include <thread>

using namespace std;


class Pose{
    public:

    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    Pose(Eigen::Vector3d po, Eigen::Quaterniond qu): p(po), q(qu) {}
    Pose(): p(Eigen::Vector3d::Zero()), q(Eigen::Quaterniond::Identity()) {}

    Pose diff(const Pose ref) const {
        Eigen::Vector3d newp = ref.q.conjugate() * (p-ref.p);
        Eigen::Quaterniond newq = q * ref.q.conjugate();

        return Pose(newp, newq);
    }

    Pose apply(const Pose t) const {
        Eigen::Vector3d newp = p + q*t.p;
        Eigen::Quaterniond newq = t.q*q;
        return Pose(newp, newq);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class Drone{
    public:
    int set;
    string name;

    ros::NodeHandle n;

    ros::Subscriber vins_sub;
    ros::Subscriber gps_sub;
    ros::Timer timer;

    mutex g;
    mutex v;
    Pose prev_vins_pose;
    Pose prev_gps_pose;
    Pose gps_pose;
    Pose vins_pose;

    deque<array<double, 6>> squared_error;

    Drone() {}

    Drone(const string& na):name(na){

        // initialize subscribers and publishers here
        // subscribing to: self vins, self nlinknodeframe
        // publishing pose, path, odometry
        set = 0;

        vins_sub = n.subscribe("/"+name+"/vins_estimator/odometry", 100, &Drone::vins_callback, this);
        gps_sub = n.subscribe("/"+name+"_gps_pose", 100, &Drone::gps_callback, this);

        timer = n.createTimer(ros::Duration(0.1), &Drone::callback, this);

    }   

    void gps_callback(const nav_msgs::Odometry::ConstPtr &gps_msg){
        set = 1;
        Eigen::Matrix<double, 3, 1> p(gps_msg->pose.pose.position.x, gps_msg->pose.pose.position.y, gps_msg->pose.pose.position.z);
        Eigen::Quaterniond q(gps_msg->pose.pose.orientation.w, gps_msg->pose.pose.orientation.x, 
                        gps_msg->pose.pose.orientation.y, gps_msg->pose.pose.orientation.z);

        g.lock();
        gps_pose = Pose(p, q);
        g.unlock();
    }

    void vins_callback(const nav_msgs::Odometry::ConstPtr &vins_msg){
        Eigen::Matrix<double, 3, 1> p(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y, vins_msg->pose.pose.position.z);
        Eigen::Quaterniond q(vins_msg->pose.pose.orientation.w, vins_msg->pose.pose.orientation.x, 
                        vins_msg->pose.pose.orientation.y, vins_msg->pose.pose.orientation.z);
        v.lock();
        vins_pose = Pose(p, q);
        v.unlock();
    }

    void callback(const ros::TimerEvent&){
        if(set == 0){return;}
        v.lock();
        g.lock();

        Pose gps_diff = gps_pose.diff(prev_gps_pose);
        Pose vins_diff = vins_pose.diff(prev_vins_pose);
        Pose diff = vins_diff.diff(gps_diff);

        array<double, 6> sq_diff = {pow(vins_diff.p.x(), 2), pow(vins_diff.p.y(), 2), pow(vins_diff.p.z(), 2), 
                            pow(vins_diff.q.x(), 2), pow(vins_diff.q.y(), 2), pow(vins_diff.q.z(), 2) };
        squared_error.push_back(sq_diff);

        prev_gps_pose = gps_pose;
        prev_vins_pose = vins_pose;

        g.unlock();
        v.unlock();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "vins_to_gps");

    Drone* drone1 = new Drone("UAV1");
    Drone* drone2 = new Drone("UAV2");
    Drone* drone3 = new Drone("UAV3");
    Drone* drone4 = new Drone("UAV5");
    Drone* drone5 = new Drone("UGV1");

    ros::spin();
    array<Drone*, 5> dl = {drone1, drone2, drone3, drone4, drone5};

    for(int i = 0; i<5; i++){
        double perror_x = 0;
        double perror_y = 0;
        double perror_z = 0;
        double qerror_x = 0;
        double qerror_y = 0;
        double qerror_z = 0;

        int size = dl[i]->squared_error.size();
        std::cout << size << std::endl;

        for(int j = 0; j < size; j++){
            perror_x += dl[i]->squared_error[j][0];
            perror_y += dl[i]->squared_error[j][1];
            perror_z += dl[i]->squared_error[j][2];
            qerror_x += dl[i]->squared_error[j][3];
            qerror_y += dl[i]->squared_error[j][4];
            qerror_z += dl[i]->squared_error[j][5];
        }
        std::cout << perror_x/size << 
                ", "<< perror_y/size << 
                ", "<< perror_z/size << 
                ", "<< qerror_x/size << 
                ", "<< qerror_y/size << 
                ", "<< qerror_z/size << std::endl;
    }

}