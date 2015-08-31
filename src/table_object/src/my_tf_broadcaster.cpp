//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>

#include <cstdlib>
#include <Eigen/Geometry>

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    ros::Time now = ros::Time::now();
    float t = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    t = t*0.1-0.05;
    ros::Duration(1.0+t).sleep();
    tf::TransformBroadcaster br;
    tf::Transform tf_transform;

    tf::Quaternion tf_quaternion;
    tf_quaternion.setRPY(2.528086, 0.041017, -0.028866); // safe, actually perform YPR, don't use setEuler
    tf_transform.setOrigin( tf::Vector3(-0.125001, -0.230695, 0.953843) );
    tf_transform.setRotation( tf_quaternion );
    
    tf::Transform tf_calibration;
    tf_calibration.setIdentity();
    tf_quaternion.setRPY(0.19,0.23,0);
    tf_calibration.setRotation(tf_quaternion);
    
    tf_transform.mult(tf_transform,tf_calibration);
    
    br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "pcl_camera", "bottle_frame")); // parent, then child frame
    
    ROS_INFO("sending transform ........");
            
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(20.0);
  
  /***************************************
    *  setup publisher, subscriber
    ***************************************/
  ROS_INFO("wait for camera/depth_registered/points");
 ros::Subscriber scene_subscriber = node.subscribe("camera/depth_registered/points", 1, cloud_cb);
//   while (node.ok()){
//     transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
//     transform.setRotation( tf::Quaternion(0, 0, 0) );
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pcl_camera", "bottle_frame"));
//     rate.sleep();
//   }
 
 ros::spin();
  return 0;
};