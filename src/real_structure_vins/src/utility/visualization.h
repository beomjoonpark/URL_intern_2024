#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

extern ros::Publisher pub_visualization_uav1_point_gt;
extern ros::Publisher pub_visualization_uav2_point_gt;
extern ros::Publisher pub_visualization_uav3_point_gt;
extern ros::Publisher pub_visualization_uav5_point_gt;
extern ros::Publisher pub_visualization_ugv1_point_gt;
extern ros::Publisher pub_visualization_uav1_uav2_line_gt;
extern ros::Publisher pub_visualization_uav1_uav3_line_gt;
extern ros::Publisher pub_visualization_uav1_uav5_line_gt;
extern ros::Publisher pub_visualization_uav1_ugv1_line_gt;
extern ros::Publisher pub_visualization_uav2_uav3_line_gt;
extern ros::Publisher pub_visualization_uav2_uav5_line_gt;
extern ros::Publisher pub_visualization_uav2_ugv1_line_gt;
extern ros::Publisher pub_visualization_uav3_uav5_line_gt;
extern ros::Publisher pub_visualization_uav3_ugv1_line_gt;
extern ros::Publisher pub_visualization_uav5_ugv1_line_gt;


// Beomjoon Park modified
// begin

extern ros::Publisher pub_visualization_uav1_point_vins_aligned;
extern ros::Publisher pub_visualization_uav2_point_vins_aligned;
extern ros::Publisher pub_visualization_uav3_point_vins_aligned;
extern ros::Publisher pub_visualization_uav5_point_vins_aligned;
extern ros::Publisher pub_visualization_ugv1_point_vins_aligned;
extern ros::Publisher pub_visualization_uav1_uav2_line_vins_aligned;
extern ros::Publisher pub_visualization_uav1_uav3_line_vins_aligned;
extern ros::Publisher pub_visualization_uav1_uav5_line_vins_aligned;
extern ros::Publisher pub_visualization_uav1_ugv1_line_vins_aligned;
extern ros::Publisher pub_visualization_uav2_uav3_line_vins_aligned;
extern ros::Publisher pub_visualization_uav2_uav5_line_vins_aligned;
extern ros::Publisher pub_visualization_uav2_ugv1_line_vins_aligned;
extern ros::Publisher pub_visualization_uav3_uav5_line_vins_aligned;
extern ros::Publisher pub_visualization_uav3_ugv1_line_vins_aligned;
extern ros::Publisher pub_visualization_uav5_ugv1_line_vins_aligned;

// end


void registerPub(ros::NodeHandle &n);

///// gt /////
void pubUAV1point_gt(const visualization_msgs::Marker &marker);

void pubUAV2point_gt(const visualization_msgs::Marker &marker);

void pubUAV3point_gt(const visualization_msgs::Marker &marker);

void pubUAV5point_gt(const visualization_msgs::Marker &marker);

void pubUGV1point_gt(const visualization_msgs::Marker &marker);

void pubUAV1UAV2line_gt(const visualization_msgs::Marker &marker);

void pubUAV1UAV3line_gt(const visualization_msgs::Marker &marker);

void pubUAV1UAV5line_gt(const visualization_msgs::Marker &marker);

void pubUAV1UGV1line_gt(const visualization_msgs::Marker &marker);

void pubUAV2UAV3line_gt(const visualization_msgs::Marker &marker);

void pubUAV2UAV5line_gt(const visualization_msgs::Marker &marker);

void pubUAV2UGV1line_gt(const visualization_msgs::Marker &marker);

void pubUAV3UAV5line_gt(const visualization_msgs::Marker &marker);

void pubUAV3UGV1line_gt(const visualization_msgs::Marker &marker);

void pubUAV5UGV1line_gt(const visualization_msgs::Marker &marker);



// Beomjoon Park modified
// begin

///// vins_aligned /////
void pubUAV1point_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV2point_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV3point_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV5point_vins_aligned(const visualization_msgs::Marker &marker);

void pubUGV1point_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV1UAV2line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV1UAV3line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV1UAV5line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV1UGV1line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV2UAV3line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV2UAV5line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV2UGV1line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV3UAV5line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV3UGV1line_vins_aligned(const visualization_msgs::Marker &marker);

void pubUAV5UGV1line_vins_aligned(const visualization_msgs::Marker &marker);

// end