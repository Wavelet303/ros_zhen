/**
 * \file        set_speed_ratio.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_speed_ratio");
	ros::NodeHandle n;
	
	// set joint command timeout (default is 0.2s)
	ros::Publisher set_speed_ratio = n.advertise<std_msgs::Float64>("/robot/limb/left/set_speed_ratio", 1000);
	std_msgs::Float64 timeout;
	timeout.data = atof(argv[1]);
	
	ros::Rate loop_rate(100);
	
	while (ros::ok())
	{
		set_speed_ratio.publish(timeout);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

