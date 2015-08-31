#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "table_object/detect_touch_update.h"
#include "table_object/bottle_feature.h"
#include "ros_sec/typeDef.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <cmath>
#include <numeric>
#include <fstream>

geometry_msgs::PoseStamped l_finger_pose_wrp_bottle;
geometry_msgs::PoseStamped r_finger_pose_wrp_bottle;
table_object::bottle_feature bottle_feature;

bool l_finger_available =false;
bool r_finger_available =false;
bool bottle_available =false;

float bottle_hand_threshold = 0.02;
float bottle_table_threshold = 0.03;
float hand_table_threshold = 0.02;

void l_finger_callback(const geometry_msgs::PoseStamped l_finger_pose)
{
    l_finger_pose_wrp_bottle = l_finger_pose;
    l_finger_available = true;
}

void r_finger_callback(const geometry_msgs::PoseStamped r_finger_pose)
{
    r_finger_pose_wrp_bottle = r_finger_pose;
    r_finger_available = true;
}

void bottle_callback(const table_object::bottle_feature bottle_msg)
{
    bottle_feature = bottle_msg;
    bottle_available = true;
}

bool detect_touch(table_object::detect_touch_update::Request  &req,
         table_object::detect_touch_update::Response &res)
{
    if(l_finger_available & r_finger_available & bottle_available)
    {
        std::cout << "time delay: " << ros::Time::now()-l_finger_pose_wrp_bottle.header.stamp << std::endl;
        
        ROS_INFO("request: apply all event detectors d(bottle,hand), d(bottle,tabletop), d(hand,tabletop), d_new %d", req.detect);
        res.bottle_hand = false;
        res.bottle_tabletop = false;
        res.hand_tabletop = false;
        res.new_detector = false;
        
        //compare: distance between grippers when closed, and closed with bottle in hand
        float l_r_distance = std::sqrt(std::pow(l_finger_pose_wrp_bottle.pose.position.x-r_finger_pose_wrp_bottle.pose.position.x,2)
                                      +std::pow(l_finger_pose_wrp_bottle.pose.position.y-r_finger_pose_wrp_bottle.pose.position.y,2)
                                      +std::pow(l_finger_pose_wrp_bottle.pose.position.z-r_finger_pose_wrp_bottle.pose.position.z,2));
        std::cout <<"left-right finger distance = " << l_r_distance << std::endl;
        
        /***************************************
        *  1. detect bottle-hand touch
        ***************************************/
        shape_msgs::SolidPrimitive bottle_primitive = bottle_feature.bottle_cylinder.primitives[0];
        float radius = bottle_primitive.dimensions[1];
        float height = bottle_primitive.dimensions[0];
        
        float l_distance = std::sqrt(std::pow(l_finger_pose_wrp_bottle.pose.position.x,2)+std::pow(l_finger_pose_wrp_bottle.pose.position.y,2));
        float r_distance = std::sqrt(std::pow(r_finger_pose_wrp_bottle.pose.position.x,2)+std::pow(r_finger_pose_wrp_bottle.pose.position.y,2));
        std::cout <<"l_distance - radius = " << l_distance-radius << std::endl;
        std::cout <<"r_distance - radius = " << r_distance-radius << std::endl;
        
        std::cout <<"l touch bottle: " << bool(l_distance-radius < bottle_hand_threshold
            & std::abs(l_finger_pose_wrp_bottle.pose.position.z) < height/2+0.01) << std::endl;
        std::cout <<"r touch bottle: " << bool(r_distance-radius < bottle_hand_threshold 
            & std::abs(r_finger_pose_wrp_bottle.pose.position.z) < height/2+0.01) << std::endl;
        
        if(l_distance-radius < bottle_hand_threshold & r_distance-radius < bottle_hand_threshold
            & std::abs(l_finger_pose_wrp_bottle.pose.position.z) < height/2+0.01
            & std::abs(r_finger_pose_wrp_bottle.pose.position.z) < height/2+0.01)
            res.bottle_hand = true;
        
        /***************************************
        *  2. detect bottle-tabletop touch
        ***************************************/
        geometry_msgs::Pose bottle_pose = bottle_feature.bottle_cylinder.primitive_poses[0];
        tf::Quaternion bottle_quat(bottle_pose.orientation.x, bottle_pose.orientation.y,
                                     bottle_pose.orientation.z, bottle_pose.orientation.w);
        double bottle_roll, bottle_pitch, bottle_yaw;
        tf::Matrix3x3(bottle_quat).getRPY(bottle_roll, bottle_pitch, bottle_yaw);
        Eigen::Affine3f toTrackedBottleCoordinate; 
        pcl::getTransformation(bottle_pose.position.x,bottle_pose.position.y,bottle_pose.position.z,
                               bottle_roll, bottle_pitch, bottle_yaw, toTrackedBottleCoordinate);
        
        Eigen::Vector3f center(bottle_pose.position.x, bottle_pose.position.y, bottle_pose.position.z);
        Eigen::Vector3f center_to_bottom(0,0,-height/2);
        Eigen::Vector3f center_to_bottom_pcl;
        pcl::transformVector(center_to_bottom, center_to_bottom_pcl,toTrackedBottleCoordinate);
        
        pcl::PointXYZ bottom_pcl(center[0]+center_to_bottom_pcl[0], center[1]+center_to_bottom_pcl[1], center[2]+center_to_bottom_pcl[2]);
        float bottom_distance=pcl::pointToPlaneDistance<pcl::PointXYZ>(bottom_pcl, 
                                                                  0.00653337, 0.512359, 0.858746, -0.774985);
        
        std::cout << "bottle bottom to table distance: " << bottom_distance << std::endl;
        if(bottom_distance < bottle_table_threshold)
            res.bottle_tabletop = true;
            
        /***************************************
        *  3. detect hand-tabletop touch
        ***************************************/
        Eigen::Vector3f center_to_l_finger(l_finger_pose_wrp_bottle.pose.position.x,l_finger_pose_wrp_bottle.pose.position.y,l_finger_pose_wrp_bottle.pose.position.z);
        Eigen::Vector3f center_to_l_finger_pcl;
        pcl::transformVector(center_to_l_finger, center_to_l_finger_pcl,toTrackedBottleCoordinate);
        
        pcl::PointXYZ l_finger_pcl(center[0]+center_to_l_finger_pcl[0], center[1]+center_to_l_finger_pcl[1], center[2]+center_to_l_finger_pcl[2]);
        float l_finger_distance=pcl::pointToPlaneDistance<pcl::PointXYZ>(l_finger_pcl, 
                                                                  0.00653337, 0.512359, 0.858746, -0.774985);
        
        Eigen::Vector3f center_to_r_finger(r_finger_pose_wrp_bottle.pose.position.x,r_finger_pose_wrp_bottle.pose.position.y,r_finger_pose_wrp_bottle.pose.position.z);
        Eigen::Vector3f center_to_r_finger_pcl;
        pcl::transformVector(center_to_r_finger, center_to_r_finger_pcl,toTrackedBottleCoordinate);
        
        pcl::PointXYZ r_finger_pcl(center[0]+center_to_r_finger_pcl[0], center[1]+center_to_r_finger_pcl[1], center[2]+center_to_r_finger_pcl[2]);
        float r_finger_distance=pcl::pointToPlaneDistance<pcl::PointXYZ>(r_finger_pcl, 
                                                                  0.00653337, 0.512359, 0.858746, -0.774985);
        
        std::cout << "l_finger to table distance: " << l_finger_distance << std::endl;
        std::cout << "r_finger to table distance: " << r_finger_distance << std::endl;
        if(l_finger_distance < hand_table_threshold & r_finger_distance < hand_table_threshold)
            res.hand_tabletop = true;
            
        /***************************************
        *  4. d_new
        ***************************************/
        //based on output of matlab learned svm
        res.new_detector = false;
        res.new_detector = true;
		
		
        
        l_finger_available = false;
        r_finger_available = false;
        bottle_available = false;
        ROS_INFO("sending back response: (bottle,hand), (bottle,tabletop), (hand,tabletop): [%d, %d, %d, %d]", 
                 res.bottle_hand, res.bottle_tabletop, res.hand_tabletop, res.new_detector);
        res.succeed = true;
    }else{
        ROS_ERROR("waiting for all features to be received");
        res.succeed = false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "touch_detector_server_update");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(4);
    spinner.start();
        
    ros::ServiceServer service = node.advertiseService("detect_touch_update", detect_touch);
    ROS_INFO("Ready to apply touch detectors + newly created event detector.");
    
    ros::Subscriber l_finger_subscriber = node.subscribe("ll_finger_pose_wrt_bottle", 1, l_finger_callback);
    ros::Subscriber r_finger_subscriber = node.subscribe("lr_finger_pose_wrt_bottle", 1, r_finger_callback);
    ros::Subscriber bottle_subscriber = node.subscribe("bottle_feature", 1, bottle_callback);
    
//     ros::MultiThreadedSpinner spinner;
//     spinner.spin();
//     ros::spin();
    ros::waitForShutdown();

    return 0;
}
