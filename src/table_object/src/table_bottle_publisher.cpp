#include <ros/ros.h>
#include <tf/tf.h>

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>

// pcl pointcloud to pointcloud2 conversions
#include <pcl/conversions.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


// pointcloud2 to ros msgs conversions
#include <pcl_conversions/pcl_conversions.h>

// from SEC
#include "ros_sec/typeDef.h"

moveit_msgs::CollisionObject collision_object;
ros::Publisher collision_object_publisher;

bool beginning = true;

ros::Publisher pub;

pcl::visualization::PCLVisualizer result_viewer("collision objects");

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    //Transform
    pcl::PCLPointCloud2 tempPCL;
    pcl_conversions::toPCL(input, tempPCL);
//    std::string fields = pcl::getFieldsList(tempPCL);
//    ROS_INFO("sensor msgs fields: %s", fields.c_str());

    CloudPtr scene_cloud(new Cloud);
    pcl::fromPCLPointCloud2(tempPCL, *scene_cloud);
    

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbd_secne(scene_cloud);
    
    if(beginning){
     // localize the static table and the initial pose of the bottle
     beginning = false;
     
     result_viewer.addPointCloud<RefPointType>(scene_cloud, rgbd_secne, "scene");
     
    //         collision_object_publisher.publish(collision_object);
     
    }else {

     result_viewer.updatePointCloud<RefPointType>(scene_cloud, rgbd_secne, "scene");
    } 
    
    result_viewer.spinOnce (100);
    
//     sensor_msgs::PointCloud2 output;
//     // Publish the data
//     pub.publish (output);
}

int main(int argc, char **argv)
{
    result_viewer.addCoordinateSystem();

    ros::init(argc, argv, "table_bottle_publisher");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(20);   //loop rate for any loops appear in this cpp
    
    collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    /*
     * Initialize collision_object
     */
    // Adding table as Box & bottle as Cylinder primitive as collision objects
    collision_object.header.frame_id = "/base";

    /* The id of the object is used to identify it. */
    collision_object.id = "table_bottle";
    
    /* Define a table to add to the world. */
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.6; //fixed X
    table_primitive.dimensions[1] = 1.5; //fixed Y
    table_primitive.dimensions[2] = 1.0; //fixed Z

    /* A pose for the table (specified relative to frame_id) */
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 1;
    table_pose.position.y = 0.4;
    table_pose.position.z = -0.5;

    /* Define a cylinder to add to the world. */
    shape_msgs::SolidPrimitive bottle_primitive;
    bottle_primitive.type = bottle_primitive.CYLINDER;
    bottle_primitive.dimensions.resize(2);
    bottle_primitive.dimensions[0] = 0.2; //fixed height
    bottle_primitive.dimensions[1] = 0.03; //fixed radius

    /* A pose for the bottle (specified relative to frame_id) */
    geometry_msgs::Pose bottle_pose;
    bottle_pose.position.x = 0.9;
    bottle_pose.position.y = 0.4;
    bottle_pose.position.z = 0.1;

    collision_object.primitives.push_back(table_primitive);
    collision_object.primitives.push_back(bottle_primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.primitive_poses.push_back(bottle_pose);    
    collision_object.operation = collision_object.ADD;
    
    //while(collision_object_publisher.getNumSubscribers() < 1)
    //{
    //    ros::WallDuration sleep_t(0.5);
    //    sleep_t.sleep();    
    //}
    
    // Locate the table and bottle
    ros::Subscriber scene_subscriber = node_handle.subscribe("camera/depth_registered/points", 100, cloud_cb);
    pub = node_handle.advertise<sensor_msgs::PointCloud2>("output", 1);
    
    ros::spin();
//     while (ros::ok())
//     {
//         ros::spinOnce();
//     }
    //sleep(2.0);
    
//     tf::Quaternion ori;
//     geometry_msgs::Quaternion ori_quat;
//     
//     float count = 0;
//     while (ros::ok())
//     {
//         collision_object.operation = collision_object.MOVE;
//         ori.setRPY(count, 0, 0);
//         tf::quaternionTFToMsg(ori, ori_quat);
//         bottle_pose.orientation = ori_quat;
//         //ROS_INFO("%f %f %f %f", ori_quat.x, ori_quat.y, ori_quat.z, ori_quat.w);
//         collision_object.primitive_poses[1]=bottle_pose;
//         collision_object_publisher.publish(collision_object); 
//         
// 
//         //ros::spinOnce();
//         loop_rate.sleep();
//         count=count+0.1;
//     }


    return 0;
}
