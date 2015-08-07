/**
 * \file        pi2_baxter.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>
#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"

int main(int argc, char** argv)
{
	//publish a sample joint position to pi2_baxter
	ros::init(argc, argv, "pi2_baxter");
	ros::NodeHandle n;
	
	ros::Publisher joint_position_pub = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1000);
	ros::Rate loop_rate(1);
    
    ros::Time begin = ros::Time::now();
    double time_offset = 5.0;
    
	while((ros::Time::now()).toSec()-begin<time_offset)
	{
		baxter_core_msgs::JointCommand msg;
		msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
		msg.names = {"left_s0","left_s1","left_e0","left_e1","left_w0","left_w1"}; //,"left_w2","left_gripper"};
		msg.command = {0.810955272759,-0.889685757539,-0.201837847861,1.91495434991,2.93148422081,-0.563894595801}; //,-1.44300820632,100.0};
		
		joint_position_pub.publish(msg);
	}
	
	return 0;
}

