#include "table_object/table_obj_exp.h"
 
#include <ros/ros.h>
#include <tf/tf.h>


#include <time.h>
#include <cstdlib>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_action");
    ros::NodeHandle node;
    ros::Rate r(0.5);
    
    while(ros::ok())
    {
        srand ( time(NULL) );
        int RandIndex = rand() % 2;
    
        ROS_INFO("pick random action %d", RandIndex);
        r.sleep();
    }

    
}