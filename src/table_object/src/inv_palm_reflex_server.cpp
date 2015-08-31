#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/callback_queue.h>

#include "table_object/detect_touch.h"
#include "table_object/bottle_feature.h"
#include "table_object/palm_reflex_triggered.h"
#include "table_object/palm_reflex.h"
#include "ros_sec/typeDef.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <cmath>
#include <numeric>
#include <fstream>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

geometry_msgs::PoseStamped l_finger_pose_wrp_bottle;
geometry_msgs::PoseStamped r_finger_pose_wrp_bottle;
table_object::bottle_feature bottle_feature;

bool l_finger_available =false;
bool r_finger_available =false;
bool bottle_available =false;

//float bottle_hand_threshold = 0.01;
float bottle_near_hand_threshold = 0.041;
float bottle_near_gripper_threshold = 0.09;

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

// bool palm_reflex(table_object::palm_reflex::Request  &req,
//          table_object::palm_reflex::Response &res)
// {
//     if(l_finger_available & r_finger_available & bottle_available)
//     {
//         
//     }
//     return true;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "palm_reflex_server");
    ros::NodeHandle node;
    ros::Rate rate(5);
        
//     ros::ServiceServer service = node.advertiseService("palm_reflex", palm_reflex);
//     ROS_INFO("Ready to monitor and trigger palm reflex when needed");
    
    ros::Subscriber l_finger_subscriber = node.subscribe("ll_finger_pose_wrt_bottle", 1, l_finger_callback);
    ros::Subscriber r_finger_subscriber = node.subscribe("lr_finger_pose_wrt_bottle", 1, r_finger_callback);
    ros::Subscriber bottle_subscriber = node.subscribe("bottle_feature", 1, bottle_callback);
    ros::Publisher gripper_command_publisher = node.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
    
    ros::Publisher collision_object_publisher = node.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    ros::Publisher attached_object_publisher = node.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    ros::Publisher trigger_publisher = node.advertise<table_object::palm_reflex_triggered>("palm_reflex_triggered", 1);
    
    tf::TransformListener listener(ros::Duration(10));
    bool bottle_frame_added = false;
    bool palm_reflex = false;
    
    baxter_core_msgs::EndEffectorCommand gripper_command;

    while (node.ok())
    {
        ros::getGlobalCallbackQueue()->callAvailable();
        if(l_finger_available & r_finger_available & bottle_available)
        {
            tf::StampedTransform transform;
            try{
                
                listener.waitForTransform("/bottle_frame", "/reference/left_gripper", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("/bottle_frame", "/reference/left_gripper", ros::Time(0), transform);
                tf::Vector3 left_gripper_loc(transform.getOrigin()[0], transform.getOrigin()[1], transform.getOrigin()[2]);
                
                shape_msgs::SolidPrimitive bottle_primitive = bottle_feature.bottle_cylinder.primitives[0];
                float radius = bottle_primitive.dimensions[1];
                float height = bottle_primitive.dimensions[0];
                
                float l_distance = std::sqrt(std::pow(l_finger_pose_wrp_bottle.pose.position.x,2)+std::pow(l_finger_pose_wrp_bottle.pose.position.y,2));
                float r_distance = std::sqrt(std::pow(r_finger_pose_wrp_bottle.pose.position.x,2)+std::pow(r_finger_pose_wrp_bottle.pose.position.y,2));
                float gripper_distance = std::sqrt(std::pow(left_gripper_loc[0],2)+std::pow(left_gripper_loc[1],2));
                std::cout <<"l_distance - radius = " << l_distance-radius << std::endl;
                std::cout <<"r_distance - radius = " << r_distance-radius << std::endl;
                std::cout <<"gripper_base_distance - radius = " << gripper_distance-radius << std::endl;
                
                std::cout <<"hand near bottle: " << bool(l_distance-radius + r_distance-radius < bottle_near_hand_threshold) << std::endl;
                std::cout <<"gripper_base near bottle: " << bool(gripper_distance-radius < bottle_near_gripper_threshold 
                    & std::abs(left_gripper_loc[2]) < height/2 + 0.06) << std::endl;
                
                
                /*if(l_distance-radius + r_distance-radius < bottle_near_hand_threshold
                    & gripper_distance-radius < bottle_near_gripper_threshold
                    & std::abs(left_gripper_loc[2]) < height/2 + 0.06 ) */
                if(gripper_distance-radius < bottle_near_gripper_threshold)
                    palm_reflex = true;
                    
                std::cout << "palm reflex triggered: " << palm_reflex << std::endl;
                if(true)
                {
                    /*gripper_command.id = 65664;
                    gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
                    char args_buffer[50];
                    std::string position_str("position");
                    sprintf(args_buffer, "{\"%s\": 0.0}", position_str.c_str());
                    gripper_command.args = std::string(args_buffer);
                    gripper_command_publisher.publish(gripper_command);*/
                    
                    table_object::palm_reflex_triggered signal;
                    signal.palm_reflex_triggered = false;
                    trigger_publisher.publish(signal);
                    
                    //remove bottle from collision object
                    /*moveit_msgs::CollisionObject remove_bottle;
                    remove_bottle.id = "bottle";
                    remove_bottle.header.frame_id = "/pcl_camera";  
                    remove_bottle.operation = remove_bottle.REMOVE;
                    collision_object_publisher.publish(remove_bottle);*/
                    
                    //attach bottle to robot gripper
                    /*moveit_msgs::AttachedCollisionObject attached_object;
                    attached_object.link_name = "/reference/left_wrist";
                    attached_object.object.header.frame_id = "bottle_frame";
                    attached_object.object.id = "bottle";
                    geometry_msgs::Pose pose;
                    shape_msgs::SolidPrimitive primitive;
                    primitive.type = primitive.CYLINDER;  
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 0.15;
                    primitive.dimensions[1] = 0.03; 
                    attached_object.object.primitives.push_back(primitive);
                    attached_object.object.primitive_poses.push_back(pose);
                    attached_object.object.operation = attached_object.object.ADD;
                    attached_object_publisher.publish(attached_object);*/
                }else{
                    //remove bottle from attached collision object
                    /*moveit_msgs::AttachedCollisionObject remove_attached_object;
                    remove_attached_object.link_name = "/reference/left_wrist";
                    remove_attached_object.object.header.frame_id = "bottle_frame";
                    remove_attached_object.object.id = "bottle";
                    remove_attached_object.object.operation = remove_attached_object.object.REMOVE;
                    attached_object_publisher.publish(remove_attached_object);*/
                    
                    /*table_object::palm_reflex_triggered signal;
                    signal.palm_reflex_triggered = false;
                    trigger_publisher.publish(signal);*/
                    
                }
                
                
                bottle_frame_added = true;
                l_finger_available = false;
                r_finger_available = false;
                bottle_available = false;
            }
            catch (tf::TransformException ex){
                if(!bottle_frame_added) ROS_ERROR("palm_reflex_server: wait for /bottle_frame to be added in tf tree");
            }
        }

        rate.sleep();
    }

    return 0;
}
