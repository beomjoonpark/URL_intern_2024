#ifndef STRUCTURE_H
#define STRUCTURE_H

#include "Eigen/Dense"
#include <deque>
#include "mutex"

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

#include "nlink_parser/LinktrackNodeframe2.h"
#include "nlink_parser/LinktrackNode2.h"

using namespace std;
#define NUM_WINDOWS 20

const int id_to_index[10] = {-1, 0, 1, 2, -1, 3, -1, -1, -1, 4};

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

class Window{
    public:
    Pose m_pose[5];
    Pose init;
    deque<Pose> pose[5] = {deque<Pose>(), deque<Pose>(), deque<Pose>(), deque<Pose>(), deque<Pose>()};
    deque<float*> range_queue = deque<float*>();
    deque<Pose> pose_diff[5] = {deque<Pose>(), deque<Pose>(), deque<Pose>(), deque<Pose>(), deque<Pose>()};;
    deque<ros::Time> time_queue;

    Window(Pose init1, Pose init2, Pose init3, Pose init4, Pose init5, Pose init_)
    : m_pose({init_.inv().mul(init1), init_.inv().mul(init2), init_.inv().mul(init3), 
            init_.inv().mul(init4), init_.inv().mul(init5)}), init(init_){


    }

    Window() {}

    int size(){
        int s = range_queue.size();
        return s;
    }
};


/*
 * name
 * window
 * vins_buff
 * vins_buff_prev
 * range_buf[10]
 * range_lock, vins_lock
 */
class Drone{
    public:
    int id;
    int idx;
    string name;
    ros::Time time;

    Window* window;

    Pose vins_buff;
    Pose vins_buff_prev[5];
    float range_buff[5];

    mutex range_lock, vins_lock;

    nav_msgs::Path path;


    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    ros::Publisher odom_pub;

    ros::Subscriber vins_sub;
    ros::Subscriber range_sub;
    Drone() {}

    Drone(const string& na, const int d, Window* w):name(na), id(d), idx(id_to_index[id]), window(w) {

        // initialize subscribers and publishers here
        // subscribing to: self vins, self nlinknodeframe
        // publishing pose, path, odometry

        vins_sub = n.subscribe("/"+name+"/vins_estimator/odometry", 100, &Drone::vins_callback, this);
        range_sub = n.subscribe("/"+name+"/nlink_linktrack_nodeframe2", 100, &Drone::range_callback, this);

        pose_pub = n.advertise<geometry_msgs::PoseStamped>("/"+name+"/vins_optimized/pose", 100);
        odom_pub = n.advertise<nav_msgs::Odometry>("/"+name+"/vins_optimized/odometry", 100);
        path_pub = n.advertise<nav_msgs::Path>("/"+name+"/vins_optimized/path", 100);
    }   

    void range_callback(const nlink_parser::LinktrackNodeframe2::ConstPtr &range_msg){
        range_lock.lock();
        for(int i = 0; i < 5; i++) {
            range_buff[i] = 0.0; // initialization
        }
        for(int i = 0; i < range_msg->nodes.size(); i++)
        {
            int ID = range_msg->nodes[i].id;
            int idx = id_to_index[ID];
            if(idx != -1){
                range_buff[idx] = range_msg->nodes[i].dis;
            }
            // no filtering applied
        }
        range_lock.unlock();
    }

    void vins_callback(const nav_msgs::Odometry::ConstPtr &vins_msg){
        vins_lock.lock();
        Eigen::Vector3d p(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y, vins_msg->pose.pose.position.z);
        Eigen::Quaterniond q(vins_msg->pose.pose.orientation.w, vins_msg->pose.pose.orientation.x, 
                            vins_msg->pose.pose.orientation.y, vins_msg->pose.pose.orientation.z );
        Pose vins(p, q);
        vins_buff = vins;
        time = vins_msg->header.stamp;
        vins_lock.unlock();
    }
};

#endif