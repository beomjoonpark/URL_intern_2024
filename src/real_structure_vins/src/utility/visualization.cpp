#include "visualization.h"

ros::Publisher pub_visualization_uav1_point_gt;
ros::Publisher pub_visualization_uav2_point_gt;
ros::Publisher pub_visualization_uav3_point_gt;
ros::Publisher pub_visualization_uav5_point_gt;
ros::Publisher pub_visualization_ugv1_point_gt;
ros::Publisher pub_visualization_uav1_uav2_line_gt;
ros::Publisher pub_visualization_uav1_uav3_line_gt;
ros::Publisher pub_visualization_uav1_uav5_line_gt;
ros::Publisher pub_visualization_uav1_ugv1_line_gt;
ros::Publisher pub_visualization_uav2_uav3_line_gt;
ros::Publisher pub_visualization_uav2_uav5_line_gt;
ros::Publisher pub_visualization_uav2_ugv1_line_gt;
ros::Publisher pub_visualization_uav3_uav5_line_gt;
ros::Publisher pub_visualization_uav3_ugv1_line_gt;
ros::Publisher pub_visualization_uav5_ugv1_line_gt;


// Beomjoon Park modified
// begin

ros::Publisher pub_visualization_uav1_point_vins_aligned;
ros::Publisher pub_visualization_uav2_point_vins_aligned;
ros::Publisher pub_visualization_uav3_point_vins_aligned;
ros::Publisher pub_visualization_uav5_point_vins_aligned;
ros::Publisher pub_visualization_ugv1_point_vins_aligned;
ros::Publisher pub_visualization_uav1_uav2_line_vins_aligned;
ros::Publisher pub_visualization_uav1_uav3_line_vins_aligned;
ros::Publisher pub_visualization_uav1_uav5_line_vins_aligned;
ros::Publisher pub_visualization_uav1_ugv1_line_vins_aligned;
ros::Publisher pub_visualization_uav2_uav3_line_vins_aligned;
ros::Publisher pub_visualization_uav2_uav5_line_vins_aligned;
ros::Publisher pub_visualization_uav2_ugv1_line_vins_aligned;
ros::Publisher pub_visualization_uav3_uav5_line_vins_aligned;
ros::Publisher pub_visualization_uav3_ugv1_line_vins_aligned;
ros::Publisher pub_visualization_uav5_ugv1_line_vins_aligned;

// end

void registerPub(ros::NodeHandle &n) {
    pub_visualization_uav1_point_gt = n.advertise<visualization_msgs::Marker>("visualization_uav1_point_gt", 100);
    pub_visualization_uav2_point_gt = n.advertise<visualization_msgs::Marker>("visualization_uav2_point_gt", 100);
    pub_visualization_uav3_point_gt = n.advertise<visualization_msgs::Marker>("visualization_uav3_point_gt", 100);
    pub_visualization_uav5_point_gt = n.advertise<visualization_msgs::Marker>("visualization_uav5_point_gt", 100);
    pub_visualization_ugv1_point_gt = n.advertise<visualization_msgs::Marker>("visualization_ugv1_point_gt", 100);
    pub_visualization_uav1_uav2_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav2_line_gt", 100);
    pub_visualization_uav1_uav3_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav3_line_gt", 100);
    pub_visualization_uav1_uav5_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav5_line_gt", 100);
    pub_visualization_uav1_ugv1_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav1_ugv1_line_gt", 100);
    pub_visualization_uav2_uav3_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav2_uav3_line_gt", 100);
    pub_visualization_uav2_uav5_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav2_uav5_line_gt", 100);
    pub_visualization_uav2_ugv1_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav2_ugv1_line_gt", 100);
    pub_visualization_uav3_uav5_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav3_uav5_line_gt", 100);
    pub_visualization_uav3_ugv1_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav3_ugv1_line_gt", 100);
    pub_visualization_uav5_ugv1_line_gt = n.advertise<visualization_msgs::Marker>("visualization_uav5_ugv1_line_gt", 100);


    // Beomjoon Park modified
    // begin

    pub_visualization_uav1_point_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav1_point_vins_aligned", 100);
    pub_visualization_uav2_point_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav2_point_vins_aligned", 100);
    pub_visualization_uav3_point_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav3_point_vins_aligned", 100);
    pub_visualization_uav5_point_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav5_point_vins_aligned", 100);
    pub_visualization_ugv1_point_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_ugv1_point_vins_aligned", 100);
    pub_visualization_uav1_uav2_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav2_line_vins_aligned", 100);
    pub_visualization_uav1_uav3_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav3_line_vins_aligned", 100);
    pub_visualization_uav1_uav5_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav1_uav5_line_vins_aligned", 100);
    pub_visualization_uav1_ugv1_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav1_ugv1_line_vins_aligned", 100);
    pub_visualization_uav2_uav3_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav2_uav3_line_vins_aligned", 100);
    pub_visualization_uav2_uav5_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav2_uav5_line_vins_aligned", 100);
    pub_visualization_uav2_ugv1_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav2_ugv1_line_vins_aligned", 100);
    pub_visualization_uav3_uav5_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav3_uav5_line_vins_aligned", 100);
    pub_visualization_uav3_ugv1_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav3_ugv1_line_vins_aligned", 100);
    pub_visualization_uav5_ugv1_line_vins_aligned = n.advertise<visualization_msgs::Marker>("visualization_uav5_ugv1_line_vins_aligned", 100);

    // end

}
 ///// gt /////
void pubUAV1point_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_point_gt.publish(marker);
}

void pubUAV2point_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_point_gt.publish(marker);
}

void pubUAV3point_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_point_gt.publish(marker);
}

void pubUAV5point_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav5_point_gt.publish(marker);
}

void pubUGV1point_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_ugv1_point_gt.publish(marker);
}

void pubUAV1UAV2line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav2_line_gt.publish(marker);
}

void pubUAV1UAV3line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav3_line_gt.publish(marker);
}

void pubUAV1UAV5line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav5_line_gt.publish(marker);
}

void pubUAV1UGV1line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_ugv1_line_gt.publish(marker);
}

void pubUAV2UAV3line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_uav3_line_gt.publish(marker);
}

void pubUAV2UAV5line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_uav5_line_gt.publish(marker);
}

void pubUAV2UGV1line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_ugv1_line_gt.publish(marker);
}

void pubUAV3UAV5line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_uav5_line_gt.publish(marker);
}

void pubUAV3UGV1line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_ugv1_line_gt.publish(marker);
}

void pubUAV5UGV1line_gt(const visualization_msgs::Marker &marker) {
    pub_visualization_uav5_ugv1_line_gt.publish(marker);
}


// Beomjoon Park modified
// begin

///// vins_aligned /////
void pubUAV1point_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_point_vins_aligned.publish(marker);
}

void pubUAV2point_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_point_vins_aligned.publish(marker);
}

void pubUAV3point_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_point_vins_aligned.publish(marker);
}

void pubUAV5point_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav5_point_vins_aligned.publish(marker);
}

void pubUGV1point_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_ugv1_point_vins_aligned.publish(marker);
}

void pubUAV1UAV2line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav2_line_vins_aligned.publish(marker);
}

void pubUAV1UAV3line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav3_line_vins_aligned.publish(marker);
}

void pubUAV1UAV5line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_uav5_line_vins_aligned.publish(marker);
}

void pubUAV1UGV1line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav1_ugv1_line_vins_aligned.publish(marker);
}

void pubUAV2UAV3line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_uav3_line_vins_aligned.publish(marker);
}

void pubUAV2UAV5line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_uav5_line_vins_aligned.publish(marker);
}

void pubUAV2UGV1line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav2_ugv1_line_vins_aligned.publish(marker);
}

void pubUAV3UAV5line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_uav5_line_vins_aligned.publish(marker);
}

void pubUAV3UGV1line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav3_ugv1_line_vins_aligned.publish(marker);
}

void pubUAV5UGV1line_vins_aligned(const visualization_msgs::Marker &marker) {
    pub_visualization_uav5_ugv1_line_vins_aligned.publish(marker);
}

// end