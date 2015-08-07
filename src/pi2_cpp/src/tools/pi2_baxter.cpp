/**
 * \file        pi2_baxter.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>
#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/EndpointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

int count = 0;
geometry_msgs::Pose average_pose;

void eeStateCallback(const baxter_core_msgs::EndpointState& msg)
{
	count++;
	std::cout << "at time stamp: " << msg.header.stamp << std::endl;
	printf("position: %f, %f, %f\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	printf("orientation: %f, %f, %f, %f\n", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	
	average_pose.position.x += msg.pose.position.x;
	average_pose.position.y += msg.pose.position.y;
	average_pose.position.z += msg.pose.position.z;
	
	average_pose.orientation.x += msg.pose.orientation.x;
	average_pose.orientation.y += msg.pose.orientation.y;
	average_pose.orientation.z += msg.pose.orientation.z;
	average_pose.orientation.w += msg.pose.orientation.w;
}

int main(int argc, char** argv)
{
	//publish a sample joint position to pi2_baxter
	ros::init(argc, argv, "pi2_baxter");
	ros::NodeHandle n;
	
	ros::Publisher joint_position_pub = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1000);
	ros::ServiceClient client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
	baxter_core_msgs::SolvePositionIK srv;
	
	//endpoint pose
	geometry_msgs::PoseStamped ee_pose;
	ee_pose.header.stamp = ros::Time::now();
	ee_pose.header.frame_id = "base";
	ee_pose.pose.position.x = 0.5;
	ee_pose.pose.position.y = 0.6;
	ee_pose.pose.position.z = 0.05;
	ee_pose.pose.orientation.x = -0.018;
	ee_pose.pose.orientation.y = 0.994;
	ee_pose.pose.orientation.z = -0.003;
	ee_pose.pose.orientation.w = 0.111;
	
	//call IKService
	srv.request.pose_stamp.push_back(ee_pose);
	if (client.call(srv))
	{
		if(srv.response.isValid[0])
		{
			ROS_INFO("SUCCESS - Valid Joint Solution Found");
			for(int i=0; i<(int)srv.response.joints[0].name.size(); i++)
				std::cout << srv.response.joints[0].name[i] << " ";
			std::cout << std::endl;
			for(int i=0; i<(int)srv.response.joints[0].position.size(); i++)
				std::cout << srv.response.joints[0].position[i] << " ";
			std::cout << std::endl;
		}else
			ROS_INFO("INVALID POSE - No Valid Joint Solution Found.");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	
	ros::Rate loop_rate(10);
    double time_offset = 5.0;
	
	baxter_core_msgs::JointCommand msg;
	msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
	msg.names = srv.response.joints[0].name;
	msg.names.push_back("left_gripper");
	msg.command = srv.response.joints[0].position;
	msg.command.push_back(100.0);
    
	
	ros::Time begin = ros::Time::now();
	while((ros::Time::now()-begin).toSec()<time_offset)
	{
		joint_position_pub.publish(msg);
		loop_rate.sleep();
	}
	
	
	ros::Subscriber sub = n.subscribe("/robot/limb/left/endpoint_state", 5, eeStateCallback);
	ros::Duration(0.1).sleep();
	clock_t exe_begin = clock();
	average_pose.position.x = 0;
	average_pose.position.y = 0;
	average_pose.position.z = 0;
	average_pose.orientation.x = 0;
	average_pose.orientation.y = 0;
	average_pose.orientation.z = 0;
	average_pose.orientation.w = 0;
	count = 0;
	while(count < 5)
	{
		std::cout << "spin at time: " << ros::Time::now() << std::endl;
		ros::spinOnce();
	}
	
	average_pose.position.x = average_pose.position.x/count;
	average_pose.position.y = average_pose.position.y/count;
	average_pose.position.z = average_pose.position.z/count;
	
	average_pose.orientation.x = average_pose.orientation.x/count;
	average_pose.orientation.y = average_pose.orientation.y/count;
	average_pose.orientation.z = average_pose.orientation.z/count;
	average_pose.orientation.w = average_pose.orientation.w/count;
	
	std::cout << "************************* average pose **************************\n";
	printf("position: %f, %f, %f\n", average_pose.position.x, average_pose.position.y, average_pose.position.z);
	printf("orientation: %f, %f, %f, %f\n", average_pose.orientation.x, average_pose.orientation.y, average_pose.orientation.z, average_pose.orientation.w);
	
	clock_t exe_end = clock();
	double elapsed_secs = double(exe_end - exe_begin) / CLOCKS_PER_SEC;
	std::cout << "listen current endpoint states elapsed time: " << elapsed_secs << std::endl;
	
	return 0;
}

