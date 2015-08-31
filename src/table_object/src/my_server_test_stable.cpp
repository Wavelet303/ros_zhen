#include "ros/ros.h"
#include "table_object/record_feature.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "table_object/bottle_feature.h"

#include <cmath>
#include <numeric>

geometry_msgs::PoseStamped l_finger_pose_wrp_bottle;
geometry_msgs::PoseStamped r_finger_pose_wrp_bottle;
table_object::bottle_feature bottle_feature;

geometry_msgs::PoseStamped _pre_l_finger_pose_wrp_bottle;
geometry_msgs::PoseStamped _pre_r_finger_pose_wrp_bottle;
table_object::bottle_feature _pre_bottle_feature;

bool l_finger_available =false;
bool r_finger_available =false;
bool bottle_available =false;

// bool pre_l_finger_available =false;
// bool pre_r_finger_available =false;
// bool pre_bottle_available =false;

// bool l_finger_stable =false;
// bool r_finger_stable =false;
// bool bottle_stable =false;

// bool l_finger_enough =false;
// bool r_finger_enough =false;
// bool bottle_enough =false;

// int window = 5; // average passed #window diff
// int Window = 10;
// 
// float diff_l_finger[5];
// float diff_r_finger[5];
// float diff_bottle[5];
// 
// std::vector<table_object::bottle_feature> bottle_buffer;
// 
// int l_finger_count = 0;
// int r_finger_count = 0;
// int bottle_count = 0;
// 
// float diffPose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
// {
//     float diff_x = pose1.position.x - pose1.position.x;
//     float diff_y = pose1.position.y - pose1.position.y;
//     float diff_z = pose1.position.z - pose1.position.z;
//     float diff_qx = pose1.orientation.x - pose1.orientation.x;
//     float diff_qy = pose1.orientation.y - pose1.orientation.y;
//     float diff_qz = pose1.orientation.z - pose1.orientation.z;
//     float diff_qw = pose1.orientation.w - pose1.orientation.w;
//     
//     float diff = std::pow(diff_x,2) + std::pow(diff_y,2) + std::pow(diff_z,2) + 
//         std::pow(diff_qx,2) + std::pow(diff_qy,2) + std::pow(diff_qz,2) +std::pow(diff_qw,2);
//     diff = std::sqrt(diff);
//     
//     return diff;
// }
// 
// float diffBottleFeature(table_object::bottle_feature feature1, table_object::bottle_feature feature2)
// {
//     geometry_msgs::Pose pose1 = feature1.bottle_cylinder.primitive_poses[0];
//     geometry_msgs::Pose pose2 = feature2.bottle_cylinder.primitive_poses[0];
//     
//     float diff = diffPose(pose1, pose2);
//     return diff;
// }

void l_finger_callback(const geometry_msgs::PoseStamped l_finger_pose)
{
    l_finger_pose_wrp_bottle = l_finger_pose;
//     if(pre_l_finger_available)
//     {
//         if(!l_finger_enough & l_finger_count == window) l_finger_enough = true;
//         
//         l_finger_count = l_finger_count % window;
//         diff_l_finger[l_finger_count]=diffPose(l_finger_pose_wrp_bottle.pose, _pre_l_finger_pose_wrp_bottle.pose);
//         l_finger_count ++;
//         
//         if(l_finger_enough)
//         {
//             float average_diff = std::accumulate(diff_l_finger, diff_l_finger + window, 0) / window;
//             ROS_INFO("average_l_finger_pose_diff: %f", average_diff);
//         }
//     }
//     
//     _pre_l_finger_pose_wrp_bottle = l_finger_pose_wrp_bottle;
    
    l_finger_available = true;
//     pre_l_finger_available = true;
}

void r_finger_callback(const geometry_msgs::PoseStamped r_finger_pose)
{
    r_finger_pose_wrp_bottle = r_finger_pose;
//     if(pre_r_finger_available)
//     {
//         if(!r_finger_enough & r_finger_count == window) r_finger_enough = true;
//         
//         r_finger_count = r_finger_count % window;
//         diff_r_finger[r_finger_count]=diffPose(r_finger_pose_wrp_bottle.pose, _pre_r_finger_pose_wrp_bottle.pose);
//         r_finger_count ++;
//         
//         if(r_finger_enough)
//         {
//             float average_diff = std::accumulate(diff_r_finger, diff_r_finger + window, 0) / window;
//             ROS_INFO("average_r_finger_pose_diff: %f", average_diff);
//         }
//     }
//     
//     _pre_r_finger_pose_wrp_bottle = r_finger_pose_wrp_bottle;
    
    r_finger_available = true;
//     pre_r_finger_available = true;
}

void bottle_callback(const table_object::bottle_feature bottle_msg)
{
    bottle_feature = bottle_msg;

//     if(!bottle_enough & bottle_count == Window) bottle_enough = true;
//     
//     
//     if(bottle_enough)
//     {
//         float average_diff = diffBottleFeature(bottle_feature, bottle_buffer[0]);
//         ROS_INFO("average_bottle_msg_diff: %f", average_diff);
//         
//         bottle_buffer.erase(bottle_buffer.begin());
//     }
//         
//     bottle_buffer.push_back(bottle_feature);  
//     
//     
//     
//     bottle_count = bottle_count % Window;
//     bottle_count ++;
    
    bottle_available = true;
}

bool record_feature(table_object::record_feature::Request  &req,
         table_object::record_feature::Response &res)
{
    if(l_finger_available & r_finger_available & bottle_available)
    {
        ROS_INFO("request: %s", req.record_name.c_str());
        
        
        l_finger_available = false;
        r_finger_available = false;
        bottle_available = false;
        res.succeed = true;
        ROS_INFO("sending back response: [%d]", res.succeed);
    }else{
        ROS_ERROR("waiting for all features to be received");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_feature_server");
    ros::NodeHandle node;
    
    ros::ServiceServer service = node.advertiseService("record_feature", record_feature);
    ROS_INFO("Ready to record feature.");
    
    ros::Subscriber l_finger_subscriber = node.subscribe("ll_finger_pose_wrt_bottle", 1, l_finger_callback);
    ros::Subscriber r_finger_subscriber = node.subscribe("lr_finger_pose_wrt_bottle", 1, r_finger_callback);
    ros::Subscriber bottle_subscriber = node.subscribe("bottle_feature", 1, bottle_callback);
    
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
//     ros::spin();

    return 0;
}