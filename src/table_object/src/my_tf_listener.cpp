#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener(ros::Duration(10));

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
//         listener.waitForTransform("/reference/left_gripper", "/bottle_frame",
//                               ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/reference/left_gripper", "/bottle_frame",  
                               ros::Time(0), transform);
        ROS_INFO("received transform: %f, %f, %f, %f, %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(),
        transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    }
    catch (tf::TransformException ex){
//         ROS_ERROR("%s",ex.what());
      ROS_ERROR("wait time out...");
    }

    rate.sleep();
  }
  return 0;
};