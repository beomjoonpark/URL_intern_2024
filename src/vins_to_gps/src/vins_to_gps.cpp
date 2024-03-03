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

using namespace std;



class Pose{
    public:

    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    Pose(Eigen::Vector3d po, Eigen::Quaterniond qu): p(po), q(qu) {}
    Pose(): p(Eigen::Vector3d::Zero()), q(Eigen::Quaterniond::Identity()) {}

    Pose inv() const {
        Eigen::Vector3d newp = - (q.conjugate()*p);
        Eigen::Quaterniond newq = q.conjugate();

        return Pose(newp, newq);
    }

    Pose mul(const Pose b) const {
        Eigen::Vector3d newp = p + q*b.p;
        Eigen::Quaterniond newq = q*b.q;
        return Pose(newp, newq);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Vector3d v1(0.2839265424023872, 0.3501335000642113, -0.8898000354769099);
Eigen::Quaterniond q1(0.9696955, 0, 0, 0.2443168);
Eigen::Vector3d v2(3.6664188894947767, -5.8602600183193625, 0.6172419963716148);
Eigen::Quaterniond q2(0.9469306, 0, 0, 0.3214381);
Eigen::Vector3d v3(-2.035869980905222, 4.850016238263976, 0.15399615833327587);
Eigen::Quaterniond q3(0.9572383, 0, 0, 0.2893005);
Eigen::Vector3d v4(6.632136889366072, -9.921340631853768, 0.8212452898631724);
Eigen::Quaterniond q4(0.9556141, 0, 0, 0.2946214);
Eigen::Vector3d v5(-5.235829267022119, 10.441634395892562, 0.5710852273549998);
Eigen::Quaterniond q5(0.944544, 0, 0, 0.3283839);

Pose pose1(v1, q1);
Pose pose2(v2, q2);
Pose pose3(v3, q3);
Pose pose4(v4, q4);
Pose pose5(v5, q5);

array<Pose, 5> plist = {pose1, pose2, pose3, pose4, pose5}; 


class Drone{
    public:
    int gps_set;
    int vins_set;
    string name;
    int idx;

    int exp;

    nav_msgs::Path path;


    Pose gps_init;
    Pose vins_init;

    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    ros::Publisher odom_pub;

    ros::Subscriber vins_sub;
    ros::Subscriber gps_sub;


    Drone() {}

    Drone(const string& na, int num, int i = 0):name(na), exp(num), idx(i){

        // initialize subscribers and publishers here
        // subscribing to: self vins, self nlinknodeframe
        // publishing pose, path, odometry
        gps_set = 0;
        vins_set = 0;

        vins_sub = n.subscribe("/"+name+"/vins_estimator/odometry", 100, &Drone::vins_callback, this);
        if(num ==1){
            gps_sub = n.subscribe("/"+name+"_gps_pose", 100, &Drone::gps_callback1, this);
        }
        else if(num==2){
            gps_sub = n.subscribe("/"+name+"_gps_pose", 100, &Drone::gps_callback2, this);
        }

        pose_pub = n.advertise<geometry_msgs::PoseStamped>("/"+name+"/vins_global/pose", 100);
        odom_pub = n.advertise<nav_msgs::Odometry>("/"+name+"/vins_global/odometry", 100);
        path_pub = n.advertise<nav_msgs::Path>("/"+name+"/vins_global/path", 100);
    }   

    void gps_callback1(const nav_msgs::Odometry::ConstPtr &gps_msg){
        if(gps_set==0){
            Eigen::Matrix<double, 3, 1> p(gps_msg->pose.pose.position.x, gps_msg->pose.pose.position.y, gps_msg->pose.pose.position.z);
            Eigen::Quaterniond q(gps_msg->pose.pose.orientation.w, gps_msg->pose.pose.orientation.x, 
                            gps_msg->pose.pose.orientation.y, gps_msg->pose.pose.orientation.z);
            Pose gps_pose(p, q);
            gps_init = gps_pose;
            gps_set = 1;
        }
    }

    void gps_callback2(const geometry_msgs::PoseStamped::ConstPtr &gps_msg){
        if(gps_set==0){
            Eigen::Matrix<double, 3, 1> p(gps_msg->pose.position.x, gps_msg->pose.position.y, gps_msg->pose.position.z);
            Eigen::Quaterniond q(gps_msg->pose.orientation.w, gps_msg->pose.orientation.x, 
                            gps_msg->pose.orientation.y, gps_msg->pose.orientation.z);
            Pose gps_pose(p, q);
            gps_init = gps_pose;
            gps_set = 1;
        }
    }
    
    void vins_callback(const nav_msgs::Odometry::ConstPtr &vins_msg){
        Eigen::Matrix<double, 3, 1> p(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y, vins_msg->pose.pose.position.z);
        Eigen::Quaterniond q(vins_msg->pose.pose.orientation.w, vins_msg->pose.pose.orientation.x, 
                        vins_msg->pose.pose.orientation.y, vins_msg->pose.pose.orientation.z);
        Pose vins_pose(p, q);

        if(vins_set==0){
            vins_init = vins_pose;
            vins_set = 1;
        }
        if(gps_set && vins_set){
            Pose offset;
            if(exp == 1){
                offset = gps_init.mul(vins_init.inv());
            }
            else{
                offset = plist[idx];
            }

            Pose gvp = offset.mul(vins_pose);

            geometry_msgs::PoseStamped pose;

            pose.header.stamp = vins_msg->header.stamp;
            pose.header.frame_id = "world";
            pose.pose.position.x = gvp.p.x();
            pose.pose.position.y = gvp.p.y();
            pose.pose.position.z = gvp.p.z();
            pose.pose.orientation.x = gvp.q.x();
            pose.pose.orientation.y = gvp.q.y();
            pose.pose.orientation.z = gvp.q.z();
            pose.pose.orientation.w = gvp.q.w();
            pose_pub.publish(pose);

            nav_msgs::Odometry odom;
            odom.header.stamp = vins_msg->header.stamp;
            odom.header.frame_id = "world";
            odom.child_frame_id = "body";
            odom.pose.pose.position.x = gvp.p.x();
            odom.pose.pose.position.y = gvp.p.y();
            odom.pose.pose.position.z = gvp.p.z();
            odom.pose.pose.orientation.x = gvp.q.x();
            odom.pose.pose.orientation.y = gvp.q.y();
            odom.pose.pose.orientation.z = gvp.q.z();
            odom.pose.pose.orientation.w = gvp.q.w();
            odom_pub.publish(odom);

            path.header.stamp = ros::Time::now();
            path.header.frame_id = "world";
            path.poses.push_back(pose);
            path_pub.publish(path);

        }
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "vins_to_gps");
    if(strcmp(argv[1], "1")==0){
        Drone drone1("UAV1", 1);
        Drone drone2("UAV2", 1);
        Drone drone3("UAV3", 1);
        Drone drone4("UAV5", 1);
        Drone drone5("UGV1", 1);
        ros::spin();
    }
    else if(strcmp(argv[1], "2")==0){
        Drone drone1("UAV1", 2, 0);
        Drone drone2("UAV2", 2, 1);
        Drone drone3("UAV3", 2, 2);
        Drone drone4("UAV4", 2, 3);
        Drone drone5("UAV5", 2, 4);
        ros::spin();
    }
    // Drone drone4("UAV4", v4, q4);
    // Drone drone5("UAV5", v5, q5);
}