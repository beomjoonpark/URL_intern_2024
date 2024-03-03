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



// class Pose{
//     public:

//     Eigen::Vector3d p;
//     Eigen::Quaterniond q;

//     Pose(Eigen::Vector3d po, Eigen::Quaterniond qu): p(po), q(qu) {}
//     Pose(): p(Eigen::Vector3d::Zero()), q(Eigen::Quaterniond::Identity()) {}

//     Pose inv() const {
//         Eigen::Vector3d newp = - (q.conjugate()*p);
//         Eigen::Quaterniond newq = q.conjugate();

//         return Pose(newp, newq);
//     }

//     Pose mul(const Pose b) const {
//         Eigen::Vector3d newp = p + q*b.p;
//         Eigen::Quaterniond newq = q*b.q;
//         return Pose(newp, newq);
//     }

//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// Eigen::Vector3d v1(-0.0560235651409, 0.132127190676, 6.29819761563e-05);
// Eigen::Quaterniond q1(0.587048928033, -0.0291067839311, 0.0097672756158, 0.808969067116);
// Eigen::Vector3d v2(27.974902525, -0.0882004281685, 0.0290714094429);
// Eigen::Quaterniond q2(0.413551825949, -0.00575186623829, -0.034202364887, 0.909819763209);
// Eigen::Vector3d v3(8.56968444931, -13.3406288718, 0.0499946970182);
// Eigen::Quaterniond q3(0.293924503071, -0.00110865045294, 0.000384589097928, 0.955827918341);
// Eigen::Vector3d v4(24.8590531991, 13.6775037494, 0.0285498683626);
// Eigen::Quaterniond q4(0.900336279744, -0.0171153460221, -0.0137156303444, 0.43464184082);
// Eigen::Vector3d v5(31.0402606661, -9.21955539904, 0.0218965966532);
// Eigen::Quaterniond q5(0.90604970833, -0.0155986942046, 0.0170028846054, 0.422541724199);
// Pose pose1(v1, q1);
// Pose pose2(v2, q2);
// Pose pose3(v3, q3);
// Pose pose4(v4, q4);
// Pose pose5(v5, q5);

// array<Pose, 5> plist1 = {pose1, pose2, pose3, pose4, pose5}; 

// Eigen::Vector3d v6(-0.0123785124597, 0.00849374929637, 0.0386940839794);
// Eigen::Quaterniond q6(0.658871315646, -0.0564253819382, 0.0199359131306, 0.749871539039);
// Eigen::Vector3d v7(6.11638354535, 0.0712668810255, 0.00115638295049);
// Eigen::Quaterniond q7(0.732547158558, -0.0106439551517, -0.0108320768475, 0.680546863058);
// Eigen::Vector3d v8(-5.60222640304, -1.83490080803, -0.039282624083);
// Eigen::Quaterniond q8(0.67893518862, -0.0383182587986, -0.008059246473, 0.733153305416);
// Eigen::Vector3d v9(11.6412503051, 1.98684827932, 0.0892225850464);
// Eigen::Quaterniond q9(0.790634027917, -0.0304270381418, 0.0224104822926, 0.61112175508);
// Eigen::Vector3d v10(-11.828091146, -3.27977338912, -0.0931421601189);
// Eigen::Quaterniond q10(0.731359195992, -0.0162494787389, -0.0141017502589, 0.681653006681);
// Pose pose6(v6, q6);
// Pose pose7(v7, q7);
// Pose pose8(v8, q8);
// Pose pose9(v9, q9);
// Pose pose10(v10, q10);
// array<Pose, 5> plist2 = {pose6, pose7, pose8, pose9, pose10};

// class Drone{
//     public:
//     int gps_set;
//     int vins_set;
//     string name;
//     int idx;

//     int exp;

//     nav_msgs::Path path;


//     Pose gps_init;
//     Pose vins_init;

//     ros::NodeHandle n;
//     ros::Publisher pose_pub;
//     ros::Publisher path_pub;
//     ros::Publisher odom_pub;

//     ros::Subscriber vins_sub;
//     ros::Subscriber gps_sub;


//     Drone() {}

//     Drone(const string& na, int num, int i = 0):name(na), exp(num), idx(i){

//         // initialize subscribers and publishers here
//         // subscribing to: self vins, self nlinknodeframe
//         // publishing pose, path, odometry
//         gps_set = 0;
//         vins_set = 0;

//         vins_sub = n.subscribe("/"+name+"/vins_estimator/odometry", 100, &Drone::vins_callback, this);

//         pose_pub = n.advertise<geometry_msgs::PoseStamped>("/"+name+"/vins_modified/pose", 100);
//         odom_pub = n.advertise<nav_msgs::Odometry>("/"+name+"/vins_modified/odometry", 100);
//         path_pub = n.advertise<nav_msgs::Path>("/"+name+"/vins_modified/path", 100);
//     }   


//     void vins_callback(const nav_msgs::Odometry::ConstPtr &vins_msg){
//         Eigen::Matrix<double, 3, 1> p(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y, vins_msg->pose.pose.position.z);
//         Eigen::Quaterniond q(vins_msg->pose.pose.orientation.w, vins_msg->pose.pose.orientation.x, 
//                         vins_msg->pose.pose.orientation.y, vins_msg->pose.pose.orientation.z);
//         Pose vins_pose(p, q);

//         if(vins_set==0){
//             vins_init = vins_pose;
//             vins_set = 1;
//         }
//         if(gps_set && vins_set){
//             Pose offset;
//             if(exp == 1){
//                 offset = plist1[idx].mul(vins_init.inv());
//             }
//             else{
//                 offset = plist2[idx].mul(vins_init.inv());
//             }

//             Pose gvp = offset.mul(vins_pose);

//             geometry_msgs::PoseStamped pose;

//             pose.header.stamp = vins_msg->header.stamp;
//             pose.header.frame_id = "world";
//             pose.pose.position.x = gvp.p.x();
//             pose.pose.position.y = gvp.p.y();
//             pose.pose.position.z = gvp.p.z();
//             pose.pose.orientation.x = gvp.q.x();
//             pose.pose.orientation.y = gvp.q.y();
//             pose.pose.orientation.z = gvp.q.z();
//             pose.pose.orientation.w = gvp.q.w();
//             pose_pub.publish(pose);

//             nav_msgs::Odometry odom;
//             odom.header.stamp = vins_msg->header.stamp;
//             odom.header.frame_id = "world";
//             odom.child_frame_id = "body";
//             odom.pose.pose.position.x = gvp.p.x();
//             odom.pose.pose.position.y = gvp.p.y();
//             odom.pose.pose.position.z = gvp.p.z();
//             odom.pose.pose.orientation.x = gvp.q.x();
//             odom.pose.pose.orientation.y = gvp.q.y();
//             odom.pose.pose.orientation.z = gvp.q.z();
//             odom.pose.pose.orientation.w = gvp.q.w();
//             odom_pub.publish(odom);

//             path.header.stamp = ros::Time::now();
//             path.header.frame_id = "world";
//             path.poses.push_back(pose);
//             path_pub.publish(path);

//         }
//     }
// };


// int main(int argc, char** argv){
//     ros::init(argc, argv, "vins_to_gps");
//     if(strcmp(argv[1], "1")==0){
//         Drone drone1("UAV1", 1);
//         Drone drone2("UAV2", 1);
//         Drone drone3("UAV3", 1);
//         Drone drone4("UAV5", 1);
//         Drone drone5("UGV1", 1);
//         ros::spin();
//     }
//     else if(strcmp(argv[1], "2")==0){
//         Drone drone1("UAV1", 2, 0);
//         Drone drone2("UAV2", 2, 1);
//         Drone drone3("UAV3", 2, 2);
//         Drone drone4("UAV4", 2, 3);
//         Drone drone5("UAV5", 2, 4);
//         ros::spin();
//     }
//     // Drone drone4("UAV4", v4, q4);
//     // Drone drone5("UAV5", v5, q5);
// }

//////////////////////////////
class Drone{
    public:
    string name;


    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;


    Drone() {}

    Drone(const string& na):name(na){

        // initialize subscribers and publishers here
        // subscribing to: self vins, self nlinknodeframe
        // publishing pose, path, odometry


        sub = n.subscribe("/"+name+"_structure/vins_scaled_path_closed", 100, &Drone::callback, this);
        pub = n.advertise<geometry_msgs::PoseStamped>("/"+name+"_structure/vins_scaled_pose_closed", 100);

    }   


    void callback(const nav_msgs::Path::ConstPtr &msg){
        pub.publish(msg->poses.back());
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "pathtopose");
    if(strcmp(argv[1], "1")==0){
        Drone drone1("UAV1");
        Drone drone2("UAV2");
        Drone drone3("UAV3");
        Drone drone4("UAV5");
        Drone drone5("UGV1");
        ros::spin();
    }
    else if(strcmp(argv[1], "2")==0){
        Drone drone1("UAV1");
        Drone drone2("UAV2");
        Drone drone3("UAV3");
        Drone drone4("UAV4");
        Drone drone5("UAV5");
        ros::spin();
    }
    // Drone drone4("UAV4", v4, q4);
    // Drone drone5("UAV5", v5, q5);
}