#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "table_object/record_feature.h"
#include "table_object/bottle_feature.h"

#include <cmath>
#include <numeric>
#include <fstream>

geometry_msgs::PoseStamped l_finger_pose_wrp_bottle;
geometry_msgs::PoseStamped r_finger_pose_wrp_bottle;
table_object::bottle_feature bottle_feature;

bool l_finger_available =false;
bool r_finger_available =false;
bool bottle_available =false;


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

bool record_feature(table_object::record_feature::Request  &req,
         table_object::record_feature::Response &res)
{
    if(l_finger_available & r_finger_available & bottle_available)
    {
    	ROS_INFO("request: %s", req.record_name.c_str());
        //record features
        char feature_file[100]; // make sure it's big enough
        std::snprintf(feature_file, sizeof(feature_file), "%s_features.txt", req.record_name.c_str());
        std::ofstream feature_writer(feature_file, std::ofstream::out | std::ofstream::app);
        geometry_msgs::Pose bottle_pose = bottle_feature.bottle_cylinder.primitive_poses[0];
        shape_msgs::SolidPrimitive bottle_primitive = bottle_feature.bottle_cylinder.primitives[0];
        tf::Quaternion l_finger_quat(l_finger_pose_wrp_bottle.pose.orientation.x, l_finger_pose_wrp_bottle.pose.orientation.y,
                                     l_finger_pose_wrp_bottle.pose.orientation.z, l_finger_pose_wrp_bottle.pose.orientation.w);
        tf::Quaternion r_finger_quat(r_finger_pose_wrp_bottle.pose.orientation.x, r_finger_pose_wrp_bottle.pose.orientation.y,
                                     r_finger_pose_wrp_bottle.pose.orientation.z, r_finger_pose_wrp_bottle.pose.orientation.w);
        double l_finger_roll, l_finger_pitch, l_finger_yaw, r_finger_roll, r_finger_pitch, r_finger_yaw;
        tf::Matrix3x3(l_finger_quat).getRPY(l_finger_roll, l_finger_pitch, l_finger_yaw);
        tf::Matrix3x3(r_finger_quat).getRPY(r_finger_roll, r_finger_pitch, r_finger_yaw);
     	feature_writer << bottle_pose.position.x << " " << bottle_pose.position.y << " " << bottle_pose.position.z
				<< " " << bottle_feature.rpy.x << " " << bottle_feature.rpy.y << " " << bottle_feature.rpy.z
				<< " " << bottle_feature.color.x << " " << bottle_feature.color.y << " " << bottle_feature.color.z
				<< " " << bottle_primitive.dimensions[0] << " " << bottle_primitive.dimensions[1]
				<< " " << r_finger_pose_wrp_bottle.pose.position.x << " " << r_finger_pose_wrp_bottle.pose.position.y << " " <<r_finger_pose_wrp_bottle.pose.position.z
				<< " " << l_finger_pose_wrp_bottle.pose.position.x << " " << l_finger_pose_wrp_bottle.pose.position.y << " " <<l_finger_pose_wrp_bottle.pose.position.z
				<< " " << r_finger_roll << " " << r_finger_pitch << " " << r_finger_yaw
				<< " " << l_finger_roll << " " << l_finger_pitch << " " << l_finger_yaw
				<< std::endl;
//         feature_writer << cur_features.bottle.loc[0] << " " << cur_features.bottle.loc[1] << " " << cur_features.bottle.loc[2]
//                 << " " << cur_features.bottle.ori[0] << " " << cur_features.bottle.ori[1] << " " << cur_features.bottle.ori[2]
//                 << " " << cur_features.bottle.color[0] << " " << cur_features.bottle.color[1] << " " << cur_features.bottle.color[2]
//                 << " " << cur_features.bottle.size[0] << " " << cur_features.bottle.size[1]
//                 << " " << cur_features.gripper_1.loc[0] << " " << cur_features.gripper_1.loc[1] << " " << cur_features.gripper_1.loc[2]
//                 << " " << cur_features.gripper_2.loc[0] << " " << cur_features.gripper_2.loc[1] << " " << cur_features.gripper_2.loc[2]
//                 << std::endl;
        feature_writer.close();
        std::cout << "features saved at " << feature_file << std::endl;
        
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
