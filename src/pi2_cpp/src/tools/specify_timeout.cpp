/**
 * \file        specify_timeout.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "specify_timeout");
	ros::NodeHandle n;
	
	// set joint command timeout (default is 0.2s)
	ros::Publisher specify_timeout = n.advertise<std_msgs::Float64>("/robot/limb/left/joint_command_timeout", 1000);
	std_msgs::Float64 timeout;
	timeout.data = 1.0;
	
	ros::Rate loop_rate(100);
	
	while (ros::ok())
	{
		specify_timeout.publish(timeout);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

