/**
 * \file        dmp_recorder.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "dmp.h"
#include "ros/ros.h"
#include <iostream>
#include "baxter_core_msgs/EndpointState.h"
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <../../opt/ros/indigo/include/geometry_msgs/Pose.h>

ros::Time begin;

std::ofstream fout0;
std::ofstream fout1;
std::ofstream fout2;
std::ofstream fout3;
std::ofstream fout4;
std::ofstream fout5;
std::ofstream fout6;

std::ofstream protocol_file;

bool start;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose end_pose;
int count = 0;

void eeStateCallback(const baxter_core_msgs::EndpointState& msg)
{
	std::cout << "at time: " << (msg.header.stamp-begin).toSec() << std::endl;
	printf("position: %f, %f, %f\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	printf("orientation: %f, %f, %f, %f\n", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	
	fout0 << "   " << msg.pose.position.x;
	fout1 << "   " << msg.pose.position.y;
	fout2 << "   " << msg.pose.position.z;
	
	fout3 << "   " << msg.pose.orientation.x;
	fout4 << "   " << msg.pose.orientation.y;
	fout5 << "   " << msg.pose.orientation.z;
	fout6 << "   " << msg.pose.orientation.w;
	
	if(start)
	{
		start_pose = msg.pose;
		start = false;
	}else
		end_pose = msg.pose;
	
	count++;
}

int main(int argc, char** argv)
{
// 	DMP dmp_example(10);
// 	Eigen::MatrixXd read_T = dmp_example.readMatrix("/home/zengzhen/Desktop/kinesthetic.txt", false);
// 	std::cout << read_T << std::endl;
// 	exit(1);
	
	if(argc<2)
	{
		std::cout << "Please input DMP kinesthetic teaching folder\n";
		exit(1);
	}
	
	//initialize ros node
	ros::init(argc, argv, "dmp_Recorder");
	ros::NodeHandle n;
	
	char buffer [100];
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 0);
	fout0.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 1);
	fout1.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 2);
	fout2.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 3);
	fout3.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 4);
	fout4.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 5);
	fout5.open(buffer);
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", argv[1], 6);
	fout6.open(buffer);
	
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/protocol.txt", argv[1]);
	protocol_file.open(buffer);
	
	ros::Subscriber subscriber = n.subscribe("/robot/limb/left/endpoint_state", 100, &eeStateCallback);
	
	begin = ros::Time::now();
	start = true;
	ros::spin(); // endpoint state is published at 100Hz
	
	fout0.close();
	fout1.close();
	fout2.close();
	fout3.close();
	fout4.close();
	fout5.close();
	fout6.close();
	
	//auto generate protocol file for the recorded dmp
	double std_dev;
	int repetitions, updates, basis_noise, n_reuse;
	char cost_function[100]; //dummy variable
	
	std_dev = 20;
	repetitions = 10;
	updates = 2;
	basis_noise = 1;
	sprintf(cost_function, "acc2_exp"); 
	n_reuse = 5;
	
	protocol_file << start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z << " ";
	protocol_file << start_pose.orientation.x << " " << start_pose.orientation.y << " " << start_pose.orientation.z << " " << start_pose.orientation.w << " ";
	protocol_file << end_pose.position.x << " " << end_pose.position.y << " " << end_pose.position.z << " ";
	protocol_file << end_pose.orientation.x << " " << end_pose.orientation.y << " " << end_pose.orientation.z << " " << end_pose.orientation.w << " ";
	
	protocol_file << 0.01*count << " " << std_dev << " " << repetitions << " " << cost_function << " " << updates << " " << basis_noise << " " << n_reuse;
	
	return 0;
}

