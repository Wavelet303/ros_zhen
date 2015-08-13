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
#include <numeric>

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
char* dmp_folder_name;

// equivalent to Matlab: filter(ones(width,1)/width,1,X)
Eigen::VectorXd filter(int width, Eigen::VectorXd x)
{
	Eigen::VectorXd result;
	result.setZero(x.size());
	
	for(int i=0; i<width-1; i++)
	{
		Eigen::VectorXd temp;
		temp.setZero(i+1);
		temp.block(0,0,i+1,1) = x.block(0,0,i+1,1);
		result(i) = temp.sum()/(double)width;
	}
	
	for(int i=0; i<x.size()-width+1; i++)
	{
		Eigen::VectorXd temp;
		temp.setZero(width);
		temp.block(0,0,width,1) = x.block(i,0,width,1);
		result(i+width-1) = temp.sum()/(double)width;
	}
	
	return result;
}

// equivalent to Matlab: smooth(x, width, 'moving')
void smooth()
{
	Eigen::MatrixXd read_T;
	DMP dmp_dummy(10);
	for(int i=0; i<7; i++)
	{
		//read in recorded trajectory
		char buffer [100];
		sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d.txt", dmp_folder_name, i);
		read_T= dmp_dummy.readMatrix(buffer, false);
// 		std::cout << "read file T = " << read_T << std::endl; 
		Eigen::Map<Eigen::VectorXd> T(read_T.data(),read_T.cols()*read_T.rows(),1);
		
		//smooth trajectory
		int width = 21;
		if(T.size()<width)
			width = T.size();
		width = width-1+width%2; //force it to be odd
		
		Eigen::VectorXd c = filter(width, T);
		
		Eigen::VectorXd cbegin;
		cbegin.setZero(width-2);
		cbegin = T.block(0,0,width-2,1);
		std::vector<double> cbegin_vector(cbegin.data(), cbegin.data()+cbegin.size());
		std::vector<double> cumSum = cbegin_vector;
		std::partial_sum(cbegin_vector.begin(), cbegin_vector.end(), cumSum.begin());
		Eigen::Map<Eigen::VectorXd> cbegin_cumsum(&cumSum[0], cumSum.size());
		Eigen::VectorXd cbegin_sample;
		cbegin_sample.setZero((int)(width-1)/2);
		for(int j=0; j<cbegin_sample.size(); j++)
			cbegin_sample(j) = cbegin_cumsum(j*2)/(double)(j*2+1);
		
		Eigen::VectorXd cend;
		cend.setZero(width-2);
		cend = T.block(T.size()-width+2,0,width-2,1);
		cend = cend.reverse();
		std::vector<double> cend_vector(cend.data(), cend.data()+cend.size());
		cumSum = cend_vector;
		std::partial_sum(cend_vector.begin(), cend_vector.end(), cumSum.begin());
		Eigen::Map<Eigen::VectorXd> cend_cumsum(&cumSum[0], cumSum.size());
		Eigen::VectorXd cend_sample;
		cend_sample.setZero((int)(width-1)/2);
		for(int j=0; j<cend_sample.size(); j++)
			cend_sample(j) = cend_cumsum(cend_cumsum.size()-2*j-1)/(double)(cend_cumsum.size()-2*j);
		
		Eigen::VectorXd finalC;
		finalC.setZero(T.size());
		finalC.block(0,0,cbegin_sample.size(),1) = cbegin_sample;
		finalC.block(cbegin_sample.size(),0,T.size()-width+1,1) = c.block(width-1,0,T.size()-width+1,1);
		finalC.block(cbegin_sample.size()+T.size()-width+1, 0, cend_sample.size(),1) = cend_sample;
		
		//duplicate 1st data in trajectory
		Eigen::VectorXd finalT;
		finalT.setZero(finalC.size()+1);
		finalT.block(1,0,finalC.size(),1) = finalC;
		finalT(0) = finalT(1);
		
		//write to file
		std::ofstream smooth_file;
		sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d_smooth.txt", dmp_folder_name, i);
		smooth_file.open(buffer);
		for(int j=0; j<finalT.size(); j++)
		{
			smooth_file << "   " << finalT(j);
		}
		smooth_file.close();
	}
}

void eeStateCallback(const baxter_core_msgs::EndpointState& msg)
{
// 	std::cout << "at time: " << (msg.header.stamp-begin).toSec() << std::endl;
// 	printf("position: %f, %f, %f\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
// 	printf("orientation: %f, %f, %f, %f\n", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	
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
	
	dmp_folder_name = argv[1];
	
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
	
	//smooth data and make initial velocity to be zero by adding duplicates of the first position
	smooth();
	
	//auto generate protocol file for the recorded dmp
	double std_dev;
	int repetitions, updates, basis_noise, n_reuse;
	char cost_function[100]; //dummy variable
	
	std_dev = 20;
	repetitions = 20;
	updates = 50;
	basis_noise = 1;
	sprintf(cost_function, "acc2_exp"); 
	n_reuse = 10;
	
	protocol_file << start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z << " ";
	protocol_file << start_pose.orientation.x << " " << start_pose.orientation.y << " " << start_pose.orientation.z << " " << start_pose.orientation.w << " ";
	protocol_file << end_pose.position.x << " " << end_pose.position.y << " " << end_pose.position.z << " ";
	protocol_file << end_pose.orientation.x << " " << end_pose.orientation.y << " " << end_pose.orientation.z << " " << end_pose.orientation.w << " ";
	
	protocol_file << 0.05*count << " " << std_dev << " " << repetitions << " " << cost_function << " " << updates << " " << basis_noise << " " << n_reuse;
	
	return 0;
}

