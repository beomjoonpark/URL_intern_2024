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
// #include "range_pub.h"
// #include "real_structure.h"
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <iterator>
#include <algorithm>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
// #include "ceres_header.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>
// #include <queue>

#include "mutex"
#include "utility/visualization.h"

using namespace std;

// parameters
const int swarm_size = 10;
const int num_of_available_robot = 7;
const int num_of_using_robot = 5;
const bool use_filtering = false;

// robot id numbering
const int uav1_id = 1;
const int uav2_id = 2;
const int uav3_id = 3;
const int uav4_id = 4;
const int uav5_id = 5;
const int ugv1_id = 9;
const int ugv2_id = 8;

// sphere radius and line width for each
int n = 10;
double std_sphere_radius = 0.1 * n;
double std_line_width = 0.09 * n;
double std_alpha = 0.5;
double opt_sphere_radius = 0.2 * n; // not-used
double opt_line_width = 0.15 * n; // not-used
double opt_alpha = 0.6; // not-used
double align_sphere_radius = 0.2 * n;   // also for mirrored-option
double align_line_width = 0.15 * n;      // also for mirrored-option
double align_alpha = 0.6;           // also for mirrored-option
// color for each point(1~5) - r g b
std::vector<double> color_r = {1.0, 0.0, 0.0};
std::vector<double> color_g = {0.0, 1.0, 0.0};
std::vector<double> color_b = {0.0, 0.0, 1.0};
std::vector<double> color_y = {1.0, 1.0, 0.0};
std::vector<double> color_m = {1.0, 0.0, 1.0};
// color for each line(std, opt, opt_aligned) - r g b
std::vector<double> color_line_std = {0.5, 0.2, 0.7};
std::vector<double> color_line_opt = {0.0, 0.0, 0.0};
std::vector<double> color_line_align = {0.7, 0.5, 0.2};
std::vector<double> color_line_align_mirrored = {0.9, 0.9, 0.9};


static::mutex m_pose_uav1, m_pose_uav2, m_pose_uav3, m_pose_uav4, m_pose_uav5, m_pose_ugv1, m_pose_ugv2, m_pose_ugv3, m_pose_ugv4, m_pose_ugv5;
static::mutex m_process;

// gt data
geometry_msgs::PoseStamped uav1_pose;
// uav1_pose.header.frame_id = "world";
// uav1_pose.pose.position.x = 0.0, uav1_pose.pose.position.y = 0.0, uav1_pose.pose.position.z = 0.0;
geometry_msgs::PoseStamped uav2_pose;
// uav2_pose.pose.position.x = 0.0, uav2_pose.pose.position.y = 0.0, uav2_pose.pose.position.z = 0.0;
geometry_msgs::PoseStamped uav3_pose;
// uav3_pose.pose.position.x = 0.0, uav3_pose.pose.position.y = 0.0, uav3_pose.pose.position.z = 0.0;
geometry_msgs::PoseStamped uav5_pose;
// uav5_pose.pose.position.x = 0.0, uav5_pose.pose.position.y = 0.0, uav5_pose.pose.position.z = 0.0;
geometry_msgs::PoseStamped ugv1_pose;
// ugv1_pose.pose.position.x = 0.0, ugv1_pose.pose.position.y = 0.0, ugv1_pose.pose.position.z = 0.0;


// Beomjoon Park modified
// begin
static::mutex m_vins_uav1, m_vins_uav2, m_vins_uav3, m_vins_uav4, m_vins_uav5, m_vins_ugv1, m_vins_ugv2, m_vins_ugv3, m_vins_ugv4, m_vins_ugv5;

geometry_msgs::PoseStamped uav1_vins;
geometry_msgs::PoseStamped uav2_vins;
geometry_msgs::PoseStamped uav3_vins;
geometry_msgs::PoseStamped uav5_vins;
geometry_msgs::PoseStamped ugv1_vins;
// end

double opti_x1, opti_y1, opti_z1;
double opti_x2, opti_y2, opti_z2;
double opti_x3, opti_y3, opti_z3;
double opti_x4, opti_y4, opti_z4;
double opti_x5, opti_y5, opti_z5;

void pose_uav1_callback(const nav_msgs::Odometry::ConstPtr &pose_uav1_msg) { // gt
    m_pose_uav1.lock();
    uav1_pose.header.frame_id = "world";
    uav1_pose.pose.position.x = pose_uav1_msg->pose.pose.position.x;
    uav1_pose.pose.position.y = pose_uav1_msg->pose.pose.position.y;
    uav1_pose.pose.position.z = pose_uav1_msg->pose.pose.position.z;
    m_pose_uav1.unlock();
}

void pose_uav2_callback(const nav_msgs::Odometry::ConstPtr &pose_uav2_msg) {
    m_pose_uav2.lock();
    uav2_pose.header.frame_id = "world";
    uav2_pose.pose.position.x = pose_uav2_msg->pose.pose.position.x;
    uav2_pose.pose.position.y = pose_uav2_msg->pose.pose.position.y;
    uav2_pose.pose.position.z = pose_uav2_msg->pose.pose.position.z;
    m_pose_uav2.unlock();
}

void pose_uav3_callback(const nav_msgs::Odometry::ConstPtr &pose_uav3_msg) {
    m_pose_uav3.lock();
    uav3_pose.header.frame_id = "world";
    uav3_pose.pose.position.x = pose_uav3_msg->pose.pose.position.x;
    uav3_pose.pose.position.y = pose_uav3_msg->pose.pose.position.y;
    uav3_pose.pose.position.z = pose_uav3_msg->pose.pose.position.z;
    m_pose_uav3.unlock();
}

void pose_uav5_callback(const nav_msgs::Odometry::ConstPtr &pose_uav5_msg) {
    m_pose_uav5.lock();
    uav5_pose.header.frame_id = "world";
    uav5_pose.pose.position.x = pose_uav5_msg->pose.pose.position.x;
    uav5_pose.pose.position.y = pose_uav5_msg->pose.pose.position.y;
    uav5_pose.pose.position.z = pose_uav5_msg->pose.pose.position.z;
    m_pose_uav5.unlock();
}

void pose_ugv1_callback(const nav_msgs::Odometry::ConstPtr &pose_ugv1_msg) {
    m_pose_ugv1.lock();
    ugv1_pose.header.frame_id = "world";
    ugv1_pose.pose.position.x = pose_ugv1_msg->pose.pose.position.x;
    ugv1_pose.pose.position.y = pose_ugv1_msg->pose.pose.position.y;
    ugv1_pose.pose.position.z = pose_ugv1_msg->pose.pose.position.z;
    m_pose_ugv1.unlock();
}

// Beomjoon Park modified
// begin

void vins_uav1_callback(const geometry_msgs::PoseStamped::ConstPtr &vins_uav1_msg) {
    m_vins_uav1.lock();
    uav1_vins.header.frame_id = "world";
    uav1_vins.pose.position.x = vins_uav1_msg->pose.position.x;
    uav1_vins.pose.position.y = vins_uav1_msg->pose.position.y;
    uav1_vins.pose.position.z = vins_uav1_msg->pose.position.z;
    m_vins_uav1.unlock();
}

void vins_uav2_callback(const geometry_msgs::PoseStamped::ConstPtr &vins_uav2_msg) {
    m_vins_uav2.lock();
    uav2_vins.header.frame_id = "world";
    uav2_vins.pose.position.x = vins_uav2_msg->pose.position.x;
    uav2_vins.pose.position.y = vins_uav2_msg->pose.position.y;
    uav2_vins.pose.position.z = vins_uav2_msg->pose.position.z;
    m_vins_uav2.unlock();
}

void vins_uav3_callback(const geometry_msgs::PoseStamped::ConstPtr &vins_uav3_msg) {
    m_vins_uav3.lock();
    uav3_vins.header.frame_id = "world";
    uav3_vins.pose.position.x = vins_uav3_msg->pose.position.x;
    uav3_vins.pose.position.y = vins_uav3_msg->pose.position.y;
    uav3_vins.pose.position.z = vins_uav3_msg->pose.position.z;
    m_vins_uav3.unlock();
}

void vins_uav5_callback(const geometry_msgs::PoseStamped::ConstPtr &vins_uav5_msg) {
    m_vins_uav5.lock();
    uav5_vins.header.frame_id = "world";
    uav5_vins.pose.position.x = vins_uav5_msg->pose.position.x;
    uav5_vins.pose.position.y = vins_uav5_msg->pose.position.y;
    uav5_vins.pose.position.z = vins_uav5_msg->pose.position.z;
    m_vins_uav5.unlock();
}

void vins_ugv1_callback(const geometry_msgs::PoseStamped::ConstPtr &vins_ugv1_msg) {
    m_vins_ugv1.lock();
    ugv1_vins.header.frame_id = "world";
    ugv1_vins.pose.position.x = vins_ugv1_msg->pose.position.x;
    ugv1_vins.pose.position.y = vins_ugv1_msg->pose.position.y;
    ugv1_vins.pose.position.z = vins_ugv1_msg->pose.position.z;
    m_vins_ugv1.unlock();
}



// https://velog.io/@nabi4622/Ceres-Solver-%EC%BF%BC%ED%84%B0%EB%8B%88%EC%96%B8-%EB%A7%A4%EB%8B%88%ED%8F%B4%EB%93%9C
struct PointToPointError {
    PointToPointError(const Eigen::Vector3d &p_source, const Eigen::Vector3d &p_target) : p_source_(p_source), p_target_(p_target) {}
    template <typename T>
    bool operator()(const T* const quat_ptr, const T* const translation_ptr, T* residual) const {
        Eigen::Map<const Eigen::Quaternion<T>> quat(quat_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(translation_ptr);
        Eigen::Matrix<T, 3, 1> p_rotated = quat * p_source_.cast<T>() + translation - p_target_.cast<T>();
        residual[0] = p_rotated[0];
        residual[1] = p_rotated[1];
        residual[2] = p_rotated[2];
        return true;
    }
    static ceres::CostFunction *create(const Eigen::Vector3d &p_source, const Eigen::Vector3d &p_target) {
        return (new ceres::AutoDiffCostFunction<PointToPointError, 3, 4, 3>(new PointToPointError(p_source, p_target)));
    }
    const Eigen::Vector3d p_source_;
    const Eigen::Vector3d p_target_;
};

visualization_msgs::Marker show_point(const geometry_msgs::PoseStamped pose, double sphere_radius, double alpha, std::vector<double> color) {
    visualization_msgs::Marker visualization_point;
    visualization_point.header.frame_id = "world";
    visualization_point.pose.position.x = pose.pose.position.x;
    visualization_point.pose.position.y = pose.pose.position.y;
    visualization_point.pose.position.z = pose.pose.position.z;
    visualization_point.pose.orientation.x = 0.0;
    visualization_point.pose.orientation.y = 0.0;
    visualization_point.pose.orientation.z = 0.0;
    visualization_point.pose.orientation.w = 1.0;
    visualization_point.type = visualization_msgs::Marker::SPHERE;
    visualization_point.scale.x = sphere_radius;
    visualization_point.scale.y = sphere_radius;
    visualization_point.scale.z = sphere_radius;
    visualization_point.color.a = alpha;
    visualization_point.color.r = color[0];
    visualization_point.color.g = color[1];
    visualization_point.color.b = color[2];

    return visualization_point;
}

visualization_msgs::Marker show_point(const double point_x, const double point_y, const double point_z, double sphere_radius, double alpha, std::vector<double> color) {
    visualization_msgs::Marker visualization_point;
    visualization_point.header.frame_id = "world";
    visualization_point.pose.position.x = point_x;
    visualization_point.pose.position.y = point_y;
    visualization_point.pose.position.z = point_z;
    visualization_point.pose.orientation.x = 0.0;
    visualization_point.pose.orientation.y = 0.0;
    visualization_point.pose.orientation.z = 0.0;
    visualization_point.pose.orientation.w = 1.0;
    visualization_point.type = visualization_msgs::Marker::SPHERE;
    visualization_point.scale.x = sphere_radius;
    visualization_point.scale.y = sphere_radius;
    visualization_point.scale.z = sphere_radius;
    visualization_point.color.a = alpha;
    visualization_point.color.r = color[0];
    visualization_point.color.g = color[1];
    visualization_point.color.b = color[2];

    return visualization_point;
}

visualization_msgs::Marker show_line(const geometry_msgs::Point p1, const geometry_msgs::Point p2, double line_width, double alpha, std::vector<double> color) {
    visualization_msgs::Marker visualization_line;
    visualization_line.header.frame_id = "world";
    // visualization_line.ns = "line";
    visualization_line.action = visualization_msgs::Marker::ADD;
    visualization_line.pose.orientation.w = 1.0;
    visualization_line.type = visualization_msgs::Marker::LINE_STRIP;
    visualization_line.scale.x = line_width;
    visualization_line.color.a = alpha;
    visualization_line.color.r = color[0];
    visualization_line.color.g = color[1];
    visualization_line.color.b = color[2];
    visualization_line.points.push_back(p1);
    visualization_line.points.push_back(p2);

    return visualization_line;
}

void process()
{
    while (ros::ok())
    {
        m_process.lock();
        ROS_INFO("RELATIVE POSITION STRUCTURE START");


        int uav1_pose_data_check = 0, uav2_pose_data_check = 0, uav3_pose_data_check = 0, uav5_pose_data_check = 0, ugv1_pose_data_check = 0;
        if(uav1_pose.pose.position.x != 0.0 && uav1_pose.pose.position.y != 0.0 && uav1_pose.pose.position.z != 0.0) {
            uav1_pose_data_check = 1;
        }
        if(uav2_pose.pose.position.x != 0.0 && uav2_pose.pose.position.y != 0.0 && uav2_pose.pose.position.z != 0.0) {
            uav2_pose_data_check = 1;
        }
        if(uav3_pose.pose.position.x != 0.0 && uav3_pose.pose.position.y != 0.0 && uav3_pose.pose.position.z != 0.0) {
            uav3_pose_data_check = 1;
        }
        if(uav5_pose.pose.position.x != 0.0 && uav5_pose.pose.position.y != 0.0 && uav5_pose.pose.position.z != 0.0) {
            uav5_pose_data_check = 1;
        }
        if(ugv1_pose.pose.position.x != 0.0 && ugv1_pose.pose.position.y != 0.0 && ugv1_pose.pose.position.z != 0.0) {
            ugv1_pose_data_check = 1;
        }
        if(uav1_pose_data_check == 0 || uav2_pose_data_check == 0 || uav3_pose_data_check == 0 || uav5_pose_data_check == 0 || ugv1_pose_data_check == 0) {
            ROS_INFO("NOT ALL POSE(GT) RECEIVED");
            m_process.unlock();
            continue;
        }

        int uav1_vins_data_check = 0, uav2_vins_data_check = 0, uav3_vins_data_check = 0, uav5_vins_data_check = 0, ugv1_vins_data_check = 0;
        if(uav1_vins.pose.position.x != 0.0 && uav1_vins.pose.position.y != 0.0 && uav1_vins.pose.position.z != 0.0) {
            uav1_vins_data_check = 1;
        }
        if(uav2_vins.pose.position.x != 0.0 && uav2_vins.pose.position.y != 0.0 && uav2_vins.pose.position.z != 0.0) {
            uav2_vins_data_check = 1;
        }
        if(uav3_vins.pose.position.x != 0.0 && uav3_vins.pose.position.y != 0.0 && uav3_vins.pose.position.z != 0.0) {
            uav3_vins_data_check = 1;
        }
        if(uav5_vins.pose.position.x != 0.0 && uav5_vins.pose.position.y != 0.0 && uav5_vins.pose.position.z != 0.0) {
            uav5_vins_data_check = 1;
        }
        if(ugv1_vins.pose.position.x != 0.0 && ugv1_vins.pose.position.y != 0.0 && ugv1_vins.pose.position.z != 0.0) {
            ugv1_vins_data_check = 1;
        }
        if(uav1_vins_data_check == 0 || uav2_vins_data_check == 0 || uav3_vins_data_check == 0 || uav5_vins_data_check == 0 || ugv1_vins_data_check == 0) {
            ROS_INFO("NOT ALL POSE(VINS) RECEIVED");
            m_process.unlock();
            continue;
        }


        // visualize gt position
        visualization_msgs::Marker visualization_uav1_point_gt;
        visualization_uav1_point_gt = show_point(uav1_pose, std_sphere_radius, std_alpha, color_r);
        visualization_msgs::Marker visualization_uav2_point_gt;
        visualization_uav2_point_gt = show_point(uav2_pose, std_sphere_radius, std_alpha, color_g);
        visualization_msgs::Marker visualization_uav3_point_gt;
        visualization_uav3_point_gt = show_point(uav3_pose, std_sphere_radius, std_alpha, color_b);
        visualization_msgs::Marker visualization_uav5_point_gt;
        visualization_uav5_point_gt = show_point(uav5_pose, std_sphere_radius, std_alpha, color_y);
        visualization_msgs::Marker visualization_ugv1_point_gt;
        visualization_ugv1_point_gt = show_point(ugv1_pose, std_sphere_radius, std_alpha, color_m);

        geometry_msgs::Point p1_gt, p2_gt, p3_gt, p4_gt, p5_gt;
        p1_gt.x = uav1_pose.pose.position.x, p1_gt.y = uav1_pose.pose.position.y, p1_gt.z = uav1_pose.pose.position.z;
        p2_gt.x = uav2_pose.pose.position.x, p2_gt.y = uav2_pose.pose.position.y, p2_gt.z = uav2_pose.pose.position.z;
        p3_gt.x = uav3_pose.pose.position.x, p3_gt.y = uav3_pose.pose.position.y, p3_gt.z = uav3_pose.pose.position.z;
        p4_gt.x = uav5_pose.pose.position.x, p4_gt.y = uav5_pose.pose.position.y, p4_gt.z = uav5_pose.pose.position.z;
        p5_gt.x = ugv1_pose.pose.position.x, p5_gt.y = ugv1_pose.pose.position.y, p5_gt.z = ugv1_pose.pose.position.z;
        visualization_msgs::Marker visualization_uav1_uav2_line_gt;
        visualization_uav1_uav2_line_gt = show_line(p1_gt, p2_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav1_uav3_line_gt;
        visualization_uav1_uav3_line_gt = show_line(p1_gt, p3_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav1_uav5_line_gt;
        visualization_uav1_uav5_line_gt = show_line(p1_gt, p4_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav1_ugv1_line_gt;
        visualization_uav1_ugv1_line_gt = show_line(p1_gt, p5_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav2_uav3_line_gt;
        visualization_uav2_uav3_line_gt = show_line(p2_gt, p3_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav2_uav5_line_gt;
        visualization_uav2_uav5_line_gt = show_line(p2_gt, p4_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav2_ugv1_line_gt;
        visualization_uav2_ugv1_line_gt = show_line(p2_gt, p5_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav3_uav5_line_gt;
        visualization_uav3_uav5_line_gt = show_line(p3_gt, p4_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav3_ugv1_line_gt;
        visualization_uav3_ugv1_line_gt = show_line(p3_gt, p5_gt, std_line_width, std_alpha, color_line_std);
        visualization_msgs::Marker visualization_uav5_ugv1_line_gt;
        visualization_uav5_ugv1_line_gt = show_line(p4_gt, p5_gt, std_line_width, std_alpha, color_line_std);

        // visualize gt position
        pubUAV1point_gt(visualization_uav1_point_gt);
        pubUAV2point_gt(visualization_uav2_point_gt);
        pubUAV3point_gt(visualization_uav3_point_gt);
        pubUAV5point_gt(visualization_uav5_point_gt);
        pubUGV1point_gt(visualization_ugv1_point_gt);
        pubUAV1UAV2line_gt(visualization_uav1_uav2_line_gt);
        pubUAV1UAV3line_gt(visualization_uav1_uav3_line_gt);
        pubUAV1UAV5line_gt(visualization_uav1_uav5_line_gt);
        pubUAV1UGV1line_gt(visualization_uav1_ugv1_line_gt);
        pubUAV2UAV3line_gt(visualization_uav2_uav3_line_gt);
        pubUAV2UAV5line_gt(visualization_uav2_uav5_line_gt);
        pubUAV2UGV1line_gt(visualization_uav2_ugv1_line_gt);
        pubUAV3UAV5line_gt(visualization_uav3_uav5_line_gt);
        pubUAV3UGV1line_gt(visualization_uav3_ugv1_line_gt);
        pubUAV5UGV1line_gt(visualization_uav5_ugv1_line_gt);

        // uav1
        opti_x1 = uav1_vins.pose.position.x, opti_y1 = uav1_vins.pose.position.y, opti_z1 = uav1_vins.pose.position.z;

        // uav2
        opti_x2 = uav2_vins.pose.position.x, opti_y2 = uav2_vins.pose.position.y, opti_z2 = uav2_vins.pose.position.z;

        // uav3
        opti_x3 = uav3_vins.pose.position.x, opti_y3 = uav3_vins.pose.position.y, opti_z3 = uav3_vins.pose.position.z;

        // uav5
        opti_x4 = uav5_vins.pose.position.x, opti_y4 = uav5_vins.pose.position.y, opti_z4 = uav5_vins.pose.position.z;

        // ugv1
        opti_x5 = ugv1_vins.pose.position.x, opti_y5 = ugv1_vins.pose.position.y, opti_z5 = ugv1_vins.pose.position.z;



        // opt real range value
        // double uav1_uav2_range_opt = sqrt(pow(opti_x1 - opti_x2, 2) + pow(opti_y1 - opti_y2, 2) + pow(opti_z1 - opti_z2, 2));
        // double uav1_uav3_range_opt = sqrt(pow(opti_x1 - opti_x3, 2) + pow(opti_y1 - opti_y3, 2) + pow(opti_z1 - opti_z3, 2));
        // double uav1_uav5_range_opt = sqrt(pow(opti_x1 - opti_x4, 2) + pow(opti_y1 - opti_y4, 2) + pow(opti_z1 - opti_z4, 2));
        // double uav1_ugv1_range_opt = sqrt(pow(opti_x1 - opti_x5, 2) + pow(opti_y1 - opti_y5, 2) + pow(opti_z1 - opti_z5, 2));
        // double uav2_uav3_range_opt = sqrt(pow(opti_x2 - opti_x3, 2) + pow(opti_y2 - opti_y3, 2) + pow(opti_z2 - opti_z3, 2));
        // double uav2_uav5_range_opt = sqrt(pow(opti_x2 - opti_x4, 2) + pow(opti_y2 - opti_y4, 2) + pow(opti_z2 - opti_z4, 2));
        // double uav2_ugv1_range_opt = sqrt(pow(opti_x2 - opti_x5, 2) + pow(opti_y2 - opti_y5, 2) + pow(opti_z2 - opti_z5, 2));
        // double uav3_uav5_range_opt = sqrt(pow(opti_x3 - opti_x4, 2) + pow(opti_y3 - opti_y4, 2) + pow(opti_z3 - opti_z4, 2));
        // double uav3_ugv1_range_opt = sqrt(pow(opti_x3 - opti_x5, 2) + pow(opti_y3 - opti_y5, 2) + pow(opti_z3 - opti_z5, 2));
        // double uav5_ugv1_range_opt = sqrt(pow(opti_x4 - opti_x5, 2) + pow(opti_y4 - opti_y5, 2) + pow(opti_z4 - opti_z5, 2));


        // error value !!!!!
        // float uav1_uav2_range_error = abs(uav1_uav2_range_gt - uav1_uav2_range_opt); // measure - gt / gt * 100
        // float uav1_uav3_range_error = abs(uav1_uav3_range_gt - uav1_uav3_range_opt);
        // float uav1_uav5_range_error = abs(uav1_uav5_range_gt - uav1_uav5_range_opt);
        // float uav1_ugv1_range_error = abs(uav1_ugv1_range_gt - uav1_ugv1_range_opt);
        // float uav2_uav3_range_error = abs(uav2_uav3_range_gt - uav2_uav3_range_opt);
        // float uav2_uav5_range_error = abs(uav2_uav5_range_gt - uav2_uav5_range_opt);
        // float uav2_ugv1_range_error = abs(uav2_ugv1_range_gt - uav2_ugv1_range_opt);
        // float uav3_uav5_range_error = abs(uav3_uav5_range_gt - uav3_uav5_range_opt);
        // float uav3_ugv1_range_error = abs(uav3_ugv1_range_gt - uav3_ugv1_range_opt);
        // float uav5_ugv1_range_error = abs(uav5_ugv1_range_gt - uav5_ugv1_range_opt);

        // std_msgs::Float64 ceres_structure_residual;
        // ceres_structure_residual.data = summary.final_cost;
        // pubCeresStructureResidual(ceres_structure_residual);
        // std_msgs::Float64 range_perc_error_sum;
        // range_perc_error_sum.data = (uav1_uav2_range_error / uav1_uav2_range_gt) + (uav1_uav3_range_error / uav1_uav3_range_gt) + (uav1_uav5_range_error / uav1_uav5_range_gt) + (uav1_ugv1_range_error / uav1_ugv1_range_gt) + (uav2_uav3_range_error / uav2_uav3_range_gt) + (uav2_uav5_range_error / uav2_uav5_range_gt) + (uav2_ugv1_range_error / uav2_ugv1_range_gt) + (uav3_uav5_range_error / uav3_uav5_range_gt) + (uav3_ugv1_range_error / uav3_ugv1_range_gt) + (uav5_ugv1_range_error / uav5_ugv1_range_gt) * 100;
        // pubRangePercErrorSum(range_perc_error_sum);
        // std_msgs::Float64 range_perc_error_mean;
        // range_perc_error_mean.data = range_perc_error_sum.data / 10;
        // pubRangePercErrorMean(range_perc_error_mean);
        // std_msgs::Float64 range_m_error_sum;
        // range_m_error_sum.data = uav1_uav2_range_error + uav1_uav3_range_error + uav1_uav5_range_error + uav1_ugv1_range_error + uav2_uav3_range_error + uav2_uav5_range_error + uav2_ugv1_range_error + uav3_uav5_range_error + uav3_ugv1_range_error + uav5_ugv1_range_error;
        // pubRangeMErrorSum(range_m_error_sum);
        // std_msgs::Float64 range_m_error_mean;
        // range_m_error_mean.data = range_m_error_sum.data / 10;
        // pubRangeMErrorMean(range_m_error_mean);

        std::cout << "===========================================================" << std::endl;
        std::cout << "======================== ALIGNMENT ========================" << std::endl;
        std::cout << "===========================================================" << std::endl;
        ceres::Problem problem_align;
        ceres::LossFunction* loss_function_align = new ceres::HuberLoss(1.0);

        Eigen::Matrix4d transformation_matrix_final;
        Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
        Eigen::Vector3d translation(0.0, 0.0, 0.0);

        auto cost_function1 = PointToPointError::create(Eigen::Vector3d(opti_x1, opti_y1, opti_z1), Eigen::Vector3d(uav1_pose.pose.position.x, uav1_pose.pose.position.y, uav1_pose.pose.position.z));
        problem_align.AddResidualBlock(cost_function1, nullptr, quaternion.coeffs().data(), translation.data());
        auto cost_function2 = PointToPointError::create(Eigen::Vector3d(opti_x2, opti_y2, opti_z2), Eigen::Vector3d(uav2_pose.pose.position.x, uav2_pose.pose.position.y, uav2_pose.pose.position.z));
        problem_align.AddResidualBlock(cost_function2, nullptr, quaternion.coeffs().data(), translation.data());
        auto cost_function3 = PointToPointError::create(Eigen::Vector3d(opti_x3, opti_y3, opti_z3), Eigen::Vector3d(uav3_pose.pose.position.x, uav3_pose.pose.position.y, uav3_pose.pose.position.z));
        problem_align.AddResidualBlock(cost_function3, nullptr, quaternion.coeffs().data(), translation.data());
        auto cost_function4 = PointToPointError::create(Eigen::Vector3d(opti_x4, opti_y4, opti_z4), Eigen::Vector3d(uav5_pose.pose.position.x, uav5_pose.pose.position.y, uav5_pose.pose.position.z));
        problem_align.AddResidualBlock(cost_function4, nullptr, quaternion.coeffs().data(), translation.data());
        auto cost_function5 = PointToPointError::create(Eigen::Vector3d(opti_x5, opti_y5, opti_z5), Eigen::Vector3d(ugv1_pose.pose.position.x, ugv1_pose.pose.position.y, ugv1_pose.pose.position.z));
        problem_align.AddResidualBlock(cost_function5, nullptr, quaternion.coeffs().data(), translation.data());

        ceres::Solver::Options options_align;
        options_align.linear_solver_type = ceres::DENSE_QR;
        options_align.max_num_iterations = 3000;
        // options_align.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary_align;

        ceres::Solve(options_align, &problem_align, &summary_align);

        transformation_matrix_final.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        transformation_matrix_final.block<3, 1>(0, 3) = translation;

        Eigen::Matrix3d rotation_matrix_final = transformation_matrix_final.block<3,3>(0, 0);
        Eigen::Vector3d translation_vector_final = transformation_matrix_final.block<3,1>(0, 3);
        std::cout << "rotation_matrix: " << rotation_matrix_final << std::endl;
        std::cout << "translation_vector: " << translation_vector_final << std::endl;

        double opti_x1_tf = opti_x1 * rotation_matrix_final(0, 0) + opti_y1 * rotation_matrix_final(0, 1) + opti_z1 * rotation_matrix_final(0, 2) + translation_vector_final(0);
        double opti_y1_tf = opti_x1 * rotation_matrix_final(1, 0) + opti_y1 * rotation_matrix_final(1, 1) + opti_z1 * rotation_matrix_final(1, 2) + translation_vector_final(1);
        double opti_z1_tf = opti_x1 * rotation_matrix_final(2, 0) + opti_y1 * rotation_matrix_final(2, 1) + opti_z1 * rotation_matrix_final(2, 2) + translation_vector_final(2);
        double opti_x2_tf = opti_x2 * rotation_matrix_final(0, 0) + opti_y2 * rotation_matrix_final(0, 1) + opti_z2 * rotation_matrix_final(0, 2) + translation_vector_final(0);
        double opti_y2_tf = opti_x2 * rotation_matrix_final(1, 0) + opti_y2 * rotation_matrix_final(1, 1) + opti_z2 * rotation_matrix_final(1, 2) + translation_vector_final(1);
        double opti_z2_tf = opti_x2 * rotation_matrix_final(2, 0) + opti_y2 * rotation_matrix_final(2, 1) + opti_z2 * rotation_matrix_final(2, 2) + translation_vector_final(2);
        double opti_x3_tf = opti_x3 * rotation_matrix_final(0, 0) + opti_y3 * rotation_matrix_final(0, 1) + opti_z3 * rotation_matrix_final(0, 2) + translation_vector_final(0);
        double opti_y3_tf = opti_x3 * rotation_matrix_final(1, 0) + opti_y3 * rotation_matrix_final(1, 1) + opti_z3 * rotation_matrix_final(1, 2) + translation_vector_final(1);
        double opti_z3_tf = opti_x3 * rotation_matrix_final(2, 0) + opti_y3 * rotation_matrix_final(2, 1) + opti_z3 * rotation_matrix_final(2, 2) + translation_vector_final(2);
        double opti_x4_tf = opti_x4 * rotation_matrix_final(0, 0) + opti_y4 * rotation_matrix_final(0, 1) + opti_z4 * rotation_matrix_final(0, 2) + translation_vector_final(0);
        double opti_y4_tf = opti_x4 * rotation_matrix_final(1, 0) + opti_y4 * rotation_matrix_final(1, 1) + opti_z4 * rotation_matrix_final(1, 2) + translation_vector_final(1);
        double opti_z4_tf = opti_x4 * rotation_matrix_final(2, 0) + opti_y4 * rotation_matrix_final(2, 1) + opti_z4 * rotation_matrix_final(2, 2) + translation_vector_final(2);
        double opti_x5_tf = opti_x5 * rotation_matrix_final(0, 0) + opti_y5 * rotation_matrix_final(0, 1) + opti_z5 * rotation_matrix_final(0, 2) + translation_vector_final(0);
        double opti_y5_tf = opti_x5 * rotation_matrix_final(1, 0) + opti_y5 * rotation_matrix_final(1, 1) + opti_z5 * rotation_matrix_final(1, 2) + translation_vector_final(1);
        double opti_z5_tf = opti_x5 * rotation_matrix_final(2, 0) + opti_y5 * rotation_matrix_final(2, 1) + opti_z5 * rotation_matrix_final(2, 2) + translation_vector_final(2);

        ///// make it aligned with ground truth by ICP /////
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gt(new pcl::PointCloud<pcl::PointXYZ>); // gt
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_opt(new pcl::PointCloud<pcl::PointXYZ>); // optimized
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>); // aligned
        cloud_gt->width = 5;
        cloud_gt->height = 1;
        cloud_gt->points.resize(cloud_gt->width * cloud_gt->height);
        cloud_gt->points[0].x = uav1_pose.pose.position.x;
        cloud_gt->points[0].y = uav1_pose.pose.position.y;
        cloud_gt->points[0].z = uav1_pose.pose.position.z;
        cloud_gt->points[1].x = uav2_pose.pose.position.x;
        cloud_gt->points[1].y = uav2_pose.pose.position.y;
        cloud_gt->points[1].z = uav2_pose.pose.position.z;
        cloud_gt->points[2].x = uav3_pose.pose.position.x;
        cloud_gt->points[2].y = uav3_pose.pose.position.y;
        cloud_gt->points[2].z = uav3_pose.pose.position.z;
        cloud_gt->points[3].x = uav5_pose.pose.position.x;
        cloud_gt->points[3].y = uav5_pose.pose.position.y;
        cloud_gt->points[3].z = uav5_pose.pose.position.z;
        cloud_gt->points[4].x = ugv1_pose.pose.position.x;
        cloud_gt->points[4].y = ugv1_pose.pose.position.y;
        cloud_gt->points[4].z = ugv1_pose.pose.position.z;
        cloud_opt->width = 5;
        cloud_opt->height = 1;
        cloud_opt->points.resize(cloud_opt->width * cloud_opt->height);
        cloud_opt->points[0].x = opti_x1_tf;
        cloud_opt->points[0].y = opti_y1_tf;
        cloud_opt->points[0].z = opti_z1_tf;
        cloud_opt->points[1].x = opti_x2_tf;
        cloud_opt->points[1].y = opti_y2_tf;
        cloud_opt->points[1].z = opti_z2_tf;
        cloud_opt->points[2].x = opti_x3_tf;
        cloud_opt->points[2].y = opti_y3_tf;
        cloud_opt->points[2].z = opti_z3_tf;
        cloud_opt->points[3].x = opti_x4_tf;
        cloud_opt->points[3].y = opti_y4_tf;
        cloud_opt->points[3].z = opti_z4_tf;
        cloud_opt->points[4].x = opti_x5_tf;
        cloud_opt->points[4].y = opti_y5_tf;
        cloud_opt->points[4].z = opti_z5_tf;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(2000);
        icp.setMaxCorrespondenceDistance(20.0);
        icp.setTransformationEpsilon(0.001);
        icp.setInputSource(cloud_opt);
        icp.setInputTarget(cloud_gt);
        icp.align(*cloud_aligned);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
        Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3,3>(0, 0);
        Eigen::Vector3f translation_vector = transformation_matrix.block<3,1>(0, 3);
        std::cout << "rotation_matrix: " << rotation_matrix << std::endl;
        std::cout << "translation_vector: " << translation_vector << std::endl;

        Eigen::Matrix4f transformation_matrix_final_icp = transformation_matrix;
        Eigen::Matrix3f rotation_matrix_final_icp = rotation_matrix;
        Eigen::Vector3f translation_vector_final_icp = translation_vector;

        double opti_x1_aligned = opti_x1_tf * rotation_matrix_final_icp(0,0) + opti_y1_tf * rotation_matrix_final_icp(0,1) + opti_z1_tf * rotation_matrix_final_icp(0,2) + translation_vector_final_icp(0);
        double opti_y1_aligned = opti_x1_tf * rotation_matrix_final_icp(1,0) + opti_y1_tf * rotation_matrix_final_icp(1,1) + opti_z1_tf * rotation_matrix_final_icp(1,2) + translation_vector_final_icp(1);
        double opti_z1_aligned = opti_x1_tf * rotation_matrix_final_icp(2,0) + opti_y1_tf * rotation_matrix_final_icp(2,1) + opti_z1_tf * rotation_matrix_final_icp(2,2) + translation_vector_final_icp(2);
        double opti_x2_aligned = opti_x2_tf * rotation_matrix_final_icp(0,0) + opti_y2_tf * rotation_matrix_final_icp(0,1) + opti_z2_tf * rotation_matrix_final_icp(0,2) + translation_vector_final_icp(0);
        double opti_y2_aligned = opti_x2_tf * rotation_matrix_final_icp(1,0) + opti_y2_tf * rotation_matrix_final_icp(1,1) + opti_z2_tf * rotation_matrix_final_icp(1,2) + translation_vector_final_icp(1);
        double opti_z2_aligned = opti_x2_tf * rotation_matrix_final_icp(2,0) + opti_y2_tf * rotation_matrix_final_icp(2,1) + opti_z2_tf * rotation_matrix_final_icp(2,2) + translation_vector_final_icp(2);
        double opti_x3_aligned = opti_x3_tf * rotation_matrix_final_icp(0,0) + opti_y3_tf * rotation_matrix_final_icp(0,1) + opti_z3_tf * rotation_matrix_final_icp(0,2) + translation_vector_final_icp(0);
        double opti_y3_aligned = opti_x3_tf * rotation_matrix_final_icp(1,0) + opti_y3_tf * rotation_matrix_final_icp(1,1) + opti_z3_tf * rotation_matrix_final_icp(1,2) + translation_vector_final_icp(1);
        double opti_z3_aligned = opti_x3_tf * rotation_matrix_final_icp(2,0) + opti_y3_tf * rotation_matrix_final_icp(2,1) + opti_z3_tf * rotation_matrix_final_icp(2,2) + translation_vector_final_icp(2);
        double opti_x4_aligned = opti_x4_tf * rotation_matrix_final_icp(0,0) + opti_y4_tf * rotation_matrix_final_icp(0,1) + opti_z4_tf * rotation_matrix_final_icp(0,2) + translation_vector_final_icp(0);
        double opti_y4_aligned = opti_x4_tf * rotation_matrix_final_icp(1,0) + opti_y4_tf * rotation_matrix_final_icp(1,1) + opti_z4_tf * rotation_matrix_final_icp(1,2) + translation_vector_final_icp(1);
        double opti_z4_aligned = opti_x4_tf * rotation_matrix_final_icp(2,0) + opti_y4_tf * rotation_matrix_final_icp(2,1) + opti_z4_tf * rotation_matrix_final_icp(2,2) + translation_vector_final_icp(2);
        double opti_x5_aligned = opti_x5_tf * rotation_matrix_final_icp(0,0) + opti_y5_tf * rotation_matrix_final_icp(0,1) + opti_z5_tf * rotation_matrix_final_icp(0,2) + translation_vector_final_icp(0);
        double opti_y5_aligned = opti_x5_tf * rotation_matrix_final_icp(1,0) + opti_y5_tf * rotation_matrix_final_icp(1,1) + opti_z5_tf * rotation_matrix_final_icp(1,2) + translation_vector_final_icp(1);
        double opti_z5_aligned = opti_x5_tf * rotation_matrix_final_icp(2,0) + opti_y5_tf * rotation_matrix_final_icp(2,1) + opti_z5_tf * rotation_matrix_final_icp(2,2) + translation_vector_final_icp(2);



        // error measurement for option1, 2
        float option1_error, option2_error;
        option1_error = (sqrt(pow(uav1_pose.pose.position.x - opti_x1_aligned, 2) + pow(uav1_pose.pose.position.y - opti_y1_aligned, 2) + pow(uav1_pose.pose.position.z - opti_z1_aligned, 2)) + \
                        sqrt(pow(uav2_pose.pose.position.x - opti_x2_aligned, 2) + pow(uav2_pose.pose.position.y - opti_y2_aligned, 2) + pow(uav2_pose.pose.position.z - opti_z2_aligned, 2)) + \
                        sqrt(pow(uav3_pose.pose.position.x - opti_x3_aligned, 2) + pow(uav3_pose.pose.position.y - opti_y3_aligned, 2) + pow(uav3_pose.pose.position.z - opti_z3_aligned, 2)) + \
                        sqrt(pow(uav5_pose.pose.position.x - opti_x4_aligned, 2) + pow(uav5_pose.pose.position.y - opti_y4_aligned, 2) + pow(uav5_pose.pose.position.z - opti_z4_aligned, 2)) + \
                        sqrt(pow(ugv1_pose.pose.position.x - opti_x5_aligned, 2) + pow(ugv1_pose.pose.position.y - opti_y5_aligned, 2) + pow(ugv1_pose.pose.position.z - opti_z5_aligned, 2))) / 5;
        
        std::cout << "error : " << option1_error << std::endl;

        std_msgs::Float64 min_error;
        // std_msgs::Float64 range_perc_error;
        // std_msgs::Float64 range_m_error;
        visualization_msgs::Marker visualization_uav1_point_vins_aligned;
        visualization_msgs::Marker visualization_uav2_point_vins_aligned;
        visualization_msgs::Marker visualization_uav3_point_vins_aligned;
        visualization_msgs::Marker visualization_uav5_point_vins_aligned;
        visualization_msgs::Marker visualization_ugv1_point_vins_aligned;
        geometry_msgs::Point p1_vins_aligned, p2_vins_aligned, p3_vins_aligned, p4_vins_aligned, p5_vins_aligned;
        // double uav1_uav2_range_vins_aligned;
        // double uav1_uav3_range_vins_aligned;
        // double uav1_uav5_range_vins_aligned;
        // double uav1_ugv1_range_vins_aligned;
        // double uav2_uav3_range_vins_aligned;
        // double uav2_uav5_range_vins_aligned;
        // double uav2_ugv1_range_vins_aligned;
        // double uav3_uav5_range_vins_aligned;
        // double uav3_ugv1_range_vins_aligned;
        // double uav5_ugv1_range_vins_aligned;
        
        visualization_uav1_point_vins_aligned = show_point(opti_x1_aligned, opti_y1_aligned, opti_z1_aligned, align_sphere_radius, align_alpha, color_r);
        visualization_uav2_point_vins_aligned = show_point(opti_x2_aligned, opti_y2_aligned, opti_z2_aligned, align_sphere_radius, align_alpha, color_g);
        visualization_uav3_point_vins_aligned = show_point(opti_x3_aligned, opti_y3_aligned, opti_z3_aligned, align_sphere_radius, align_alpha, color_b);
        visualization_uav5_point_vins_aligned = show_point(opti_x4_aligned, opti_y4_aligned, opti_z4_aligned, align_sphere_radius, align_alpha, color_y);
        visualization_ugv1_point_vins_aligned = show_point(opti_x5_aligned, opti_y5_aligned, opti_z5_aligned, align_sphere_radius, align_alpha, color_m);

        p1_vins_aligned.x = opti_x1_aligned, p1_vins_aligned.y = opti_y1_aligned, p1_vins_aligned.z = opti_z1_aligned;
        p2_vins_aligned.x = opti_x2_aligned, p2_vins_aligned.y = opti_y2_aligned, p2_vins_aligned.z = opti_z2_aligned;
        p3_vins_aligned.x = opti_x3_aligned, p3_vins_aligned.y = opti_y3_aligned, p3_vins_aligned.z = opti_z3_aligned;
        p4_vins_aligned.x = opti_x4_aligned, p4_vins_aligned.y = opti_y4_aligned, p4_vins_aligned.z = opti_z4_aligned;
        p5_vins_aligned.x = opti_x5_aligned, p5_vins_aligned.y = opti_y5_aligned, p5_vins_aligned.z = opti_z5_aligned;

        // vins_aligned real range value
        // double uav1_uav2_range_vins_aligned = sqrt(pow(opti_x1_aligned - opti_x2_aligned, 2) + pow(opti_y1_aligned - opti_y2_aligned, 2) + pow(opti_z1_aligned - opti_z2_aligned, 2));
        // double uav1_uav3_range_vins_aligned = sqrt(pow(opti_x1_aligned - opti_x3_aligned, 2) + pow(opti_y1_aligned - opti_y3_aligned, 2) + pow(opti_z1_aligned - opti_z3_aligned, 2));
        // double uav1_uav5_range_vins_aligned = sqrt(pow(opti_x1_aligned - opti_x4_aligned, 2) + pow(opti_y1_aligned - opti_y4_aligned, 2) + pow(opti_z1_aligned - opti_z4_aligned, 2));
        // double uav1_ugv1_range_vins_aligned = sqrt(pow(opti_x1_aligned - opti_x5_aligned, 2) + pow(opti_y1_aligned - opti_y5_aligned, 2) + pow(opti_z1_aligned - opti_z5_aligned, 2));
        // double uav2_uav3_range_vins_aligned = sqrt(pow(opti_x2_aligned - opti_x3_aligned, 2) + pow(opti_y2_aligned - opti_y3_aligned, 2) + pow(opti_z2_aligned - opti_z3_aligned, 2));
        // double uav2_uav5_range_vins_aligned = sqrt(pow(opti_x2_aligned - opti_x4_aligned, 2) + pow(opti_y2_aligned - opti_y4_aligned, 2) + pow(opti_z2_aligned - opti_z4_aligned, 2));
        // double uav2_ugv1_range_vins_aligned = sqrt(pow(opti_x2_aligned - opti_x5_aligned, 2) + pow(opti_y2_aligned - opti_y5_aligned, 2) + pow(opti_z2_aligned - opti_z5_aligned, 2));
        // double uav3_uav5_range_vins_aligned = sqrt(pow(opti_x3_aligned - opti_x4_aligned, 2) + pow(opti_y3_aligned - opti_y4_aligned, 2) + pow(opti_z3_aligned - opti_z4_aligned, 2));
        // double uav3_ugv1_range_vins_aligned = sqrt(pow(opti_x3_aligned - opti_x5_aligned, 2) + pow(opti_y3_aligned - opti_y5_aligned, 2) + pow(opti_z3_aligned - opti_z5_aligned, 2));
        // double uav5_ugv1_range_vins_aligned = sqrt(pow(opti_x4_aligned - opti_x5_aligned, 2) + pow(opti_y4_aligned - opti_y5_aligned, 2) + pow(opti_z4_aligned - opti_z5_aligned, 2));

        min_error.data = option1_error;
        // range_perc_error.data = (option1_error - range) / range * 100;
        // range_m_error.data = option1_error - range;
        
        visualization_msgs::Marker visualization_uav1_uav2_line_vins_aligned;
        visualization_uav1_uav2_line_vins_aligned = show_line(p1_vins_aligned, p2_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav1_uav3_line_vins_aligned;
        visualization_uav1_uav3_line_vins_aligned = show_line(p1_vins_aligned, p3_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav1_uav5_line_vins_aligned;
        visualization_uav1_uav5_line_vins_aligned = show_line(p1_vins_aligned, p4_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav1_ugv1_line_vins_aligned;
        visualization_uav1_ugv1_line_vins_aligned = show_line(p1_vins_aligned, p5_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav2_uav3_line_vins_aligned;
        visualization_uav2_uav3_line_vins_aligned = show_line(p2_vins_aligned, p3_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav2_uav5_line_vins_aligned;
        visualization_uav2_uav5_line_vins_aligned = show_line(p2_vins_aligned, p4_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav2_ugv1_line_vins_aligned;
        visualization_uav2_ugv1_line_vins_aligned = show_line(p2_vins_aligned, p5_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav3_uav5_line_vins_aligned;
        visualization_uav3_uav5_line_vins_aligned = show_line(p3_vins_aligned, p4_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav3_ugv1_line_vins_aligned;
        visualization_uav3_ugv1_line_vins_aligned = show_line(p3_vins_aligned, p5_vins_aligned, align_line_width, align_alpha, color_line_align);
        visualization_msgs::Marker visualization_uav5_ugv1_line_vins_aligned;
        visualization_uav5_ugv1_line_vins_aligned = show_line(p4_vins_aligned, p5_vins_aligned, align_line_width, align_alpha, color_line_align);

        pubUAV1point_vins_aligned(visualization_uav1_point_vins_aligned);
        pubUAV2point_vins_aligned(visualization_uav2_point_vins_aligned);
        pubUAV3point_vins_aligned(visualization_uav3_point_vins_aligned);
        pubUAV5point_vins_aligned(visualization_uav5_point_vins_aligned);
        pubUGV1point_vins_aligned(visualization_ugv1_point_vins_aligned);
        pubUAV1UAV2line_vins_aligned(visualization_uav1_uav2_line_vins_aligned);
        pubUAV1UAV3line_vins_aligned(visualization_uav1_uav3_line_vins_aligned);
        pubUAV1UAV5line_vins_aligned(visualization_uav1_uav5_line_vins_aligned);
        pubUAV1UGV1line_vins_aligned(visualization_uav1_ugv1_line_vins_aligned);
        pubUAV2UAV3line_vins_aligned(visualization_uav2_uav3_line_vins_aligned);
        pubUAV2UAV5line_vins_aligned(visualization_uav2_uav5_line_vins_aligned);
        pubUAV2UGV1line_vins_aligned(visualization_uav2_ugv1_line_vins_aligned);
        pubUAV3UAV5line_vins_aligned(visualization_uav3_uav5_line_vins_aligned);
        pubUAV3UGV1line_vins_aligned(visualization_uav3_ugv1_line_vins_aligned);
        pubUAV5UGV1line_vins_aligned(visualization_uav5_ugv1_line_vins_aligned);

        // error
        // double range_error_uav1_uav2 = abs(uav1_uav2_range_opt_aligned - uav1_uav2_range_gt);
        // double range_error_uav1_uav3 = abs(uav1_uav3_range_opt_aligned - uav1_uav3_range_gt);
        // double range_error_uav1_uav5 = abs(uav1_uav5_range_opt_aligned - uav1_uav5_range_gt);
        // double range_error_uav1_ugv1 = abs(uav1_ugv1_range_opt_aligned - uav1_ugv1_range_gt);
        // double range_error_uav2_uav3 = abs(uav2_uav3_range_opt_aligned - uav2_uav3_range_gt);
        // double range_error_uav2_uav5 = abs(uav2_uav5_range_opt_aligned - uav2_uav5_range_gt);
        // double range_error_uav2_ugv1 = abs(uav2_ugv1_range_opt_aligned - uav2_ugv1_range_gt);
        // double range_error_uav3_uav5 = abs(uav3_uav5_range_opt_aligned - uav3_uav5_range_gt);
        // double range_error_uav3_ugv1 = abs(uav3_ugv1_range_opt_aligned - uav3_ugv1_range_gt);
        // double range_error_uav5_ugv1 = abs(uav5_ugv1_range_opt_aligned - uav5_ugv1_range_gt);

        m_process.unlock();

        std::chrono::milliseconds dura(100);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_structure_node");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ROS_WARN("waiting for uwb data");

    registerPub(nh);

    ros::Subscriber sub_pose_uav1 = nh.subscribe("/UAV1_gps_pose", 100, pose_uav1_callback);
    ros::Subscriber sub_pose_uav2 = nh.subscribe("/UAV2_gps_pose", 100, pose_uav2_callback);
    ros::Subscriber sub_pose_uav3 = nh.subscribe("/UAV3_gps_pose", 100, pose_uav3_callback);
    // ros::Subscriber sub_pose_uav4 = nh.subscribe("/UAV4_gps_pose", 100, pose_uav4_callback);
    ros::Subscriber sub_pose_uav5 = nh.subscribe("/UAV5_gps_pose", 100, pose_uav5_callback);
    ros::Subscriber sub_pose_ugv1 = nh.subscribe("/UGV1_gps_pose", 100, pose_ugv1_callback);
    // ros::Subscriber sub_pose_ugv2 = nh.subscribe("/UGV2_gps_pose", 100, pose_ugv2_callback);
    // ros::Subscriber sub_pose_ugv3 = nh.subscribe("/UGV3_gps_pose", 100, pose_ugv3_callback);
    // ros::Subscriber sub_pose_ugv4 = nh.subscribe("/UGV4_gps_pose", 100, pose_ugv4_callback);
    // ros::Subscriber sub_pose_ugv5 = nh.subscribe("/UGV5_gps_pose", 100, pose_ugv5_callback);


    ros::Subscriber sub_vins_uav1 = nh.subscribe("/UAV1/vins_global/pose", 100, vins_uav1_callback);
    ros::Subscriber sub_vins_uav2 = nh.subscribe("/UAV2/vins_global/pose", 100, vins_uav2_callback);
    ros::Subscriber sub_vins_uav3 = nh.subscribe("/UAV3/vins_global/pose", 100, vins_uav3_callback);
    // ros::Subscriber sub_vins_uav4 = nh.subscribe("/UAV4/vins_global/pose", 100, vins_uav4_callback);
    ros::Subscriber sub_vins_uav5 = nh.subscribe("/UAV5/vins_global/pose", 100, vins_uav5_callback);
    ros::Subscriber sub_vins_ugv1 = nh.subscribe("/UGV1/vins_global/pose", 100, vins_ugv1_callback);
    // ros::Subscriber sub_vins_ugv2 = nh.subscribe("/UGV2/vins_global/pose", 100, vins_ugv2_callback);
    // ros::Subscriber sub_vins_ugv3 = nh.subscribe("/UGV3/vins_global/pose", 100, vins_ugv3_callback);
    // ros::Subscriber sub_vins_ugv4 = nh.subscribe("/UGV4/vins_global/pose", 100, vins_ugv4_callback);
    // ros::Subscriber sub_vins_ugv5 = nh.subscribe("/UGV5/vins_global/pose", 100, vins_ugv5_callback);

    uav1_pose.pose.position.x = 0.0, uav1_pose.pose.position.y = 0.0, uav1_pose.pose.position.z = 0.0;
    uav2_pose.pose.position.x = 0.0, uav2_pose.pose.position.y = 0.0, uav2_pose.pose.position.z = 0.0;
    uav3_pose.pose.position.x = 0.0, uav3_pose.pose.position.y = 0.0, uav3_pose.pose.position.z = 0.0;
    uav5_pose.pose.position.x = 0.0, uav5_pose.pose.position.y = 0.0, uav5_pose.pose.position.z = 0.0;
    ugv1_pose.pose.position.x = 0.0, ugv1_pose.pose.position.y = 0.0, ugv1_pose.pose.position.z = 0.0;

    std::thread process_thread(process);
    ros::spin();

    return 0;
}
