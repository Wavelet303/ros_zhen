/**
 * \file test_learning.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief apply learned policy and take actions to imitate the behavior
 */
 
#include "table_object/table_obj_exp.h"
 
#include <ros/ros.h>
#include <tf/tf.h>

#include "table_object/detect_touch.h"
#include "table_object/record_feature.h"
#include "baxter_moveit/move_to_target_pose.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <time.h>
#include <cstdlib> 
 
// learned policy 
std::vector<std::vector<int> > good_poses_gripper_open; //open: 1; close: 0
std::vector<std::vector<int> > good_poses_index;

std::vector<std::vector<int> > human_demo;

bool check_qual(table_object::detect_touch::Response res, int qual_index)
{                                                           
    if(res.bottle_hand == (bool)human_demo[qual_index][0]       
        & res.bottle_tabletop == (bool)human_demo[qual_index][1]
        & res.hand_tabletop == (bool)human_demo[qual_index][2] )
        return true;
    else
        return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_learning");
	ros::NodeHandle node;
    ros::Rate r(0.5);

	// setup all clients and publisher
	ros::ServiceClient detect_client = node.serviceClient<table_object::detect_touch>("detect_touch");
    table_object::detect_touch detect_srv;
    detect_srv.request.detect = true;
    
    ros::ServiceClient move_client = node.serviceClient<baxter_moveit::move_to_target_pose>("move_to_target_pose");
    baxter_moveit::move_to_target_pose move_srv;
    
    ros::Publisher gripper_command_publisher = node.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
    baxter_core_msgs::EndEffectorCommand gripper_command;
    
    ros::ServiceClient feature_client = node.serviceClient<table_object::record_feature>("record_feature");
    table_object::record_feature feature_srv;
    feature_srv.request.record_name = "test";
    
    /***************************************
    *  qualitative representation from human_demo
    * - (bottle,hand), (bottle,tabletop), (hand,tabletop)
    ***************************************/
    std::vector<int> q0; q0.push_back(0); q0.push_back(1); q0.push_back(0);
    std::vector<int> q1; q1.push_back(1); q1.push_back(1); q1.push_back(0);
    std::vector<int> q2; q2.push_back(1); q2.push_back(0); q2.push_back(0);
    human_demo.push_back(q0);
    human_demo.push_back(q1);
    human_demo.push_back(q2);
    
    /***************************************
    *  construct explorer
    ***************************************/
    TableObjectAction::Explorer explorer(human_demo);
    int qual_index = 0;
    
    /***************************************
    *  apply learned policy
    ***************************************/
    int count=0;
    int success_count=0;
    int exp_num=10;
    bool move_srv_success = false;
    while(count < exp_num)
    {
    	bool imitation_success[2];
    	imitation_success[0] = false;
    	imitation_success[1] = false;
		for(qual_index=0; qual_index<human_demo.size(); qual_index++)
		{
		
			// random choose one 
			srand ( time(NULL) ); //initialize the random seed
			int RandIndex = rand() % good_poses_index[qual_index].size(); //generates a random number between 0 and godd_poses[qual_index].size()
		
			if(qual_index==0)
			{
				// get to home pose first
				move_srv_success = false;
				move_srv.request.target_pose=explorer.getHomePose();
				std::cout << "send target pose to server: go to home pose " << std::endl;
				while(!move_srv_success)
				{
					if (move_client.call(move_srv))
					{
						ROS_INFO("plan and execution succeed: %d", move_srv.response.succeed);
						move_srv_success = move_srv.response.succeed;
					}
					else
					{
						ROS_ERROR("Failed to call service move_to_target_pose...Retrying...");
					}
				}
			
				if(good_poses_gripper_open[qual_index][RandIndex])
				{
					gripper_command.id = 65664;
		            gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
		            char args_buffer[50];
		            std::string position_str("position");
		            sprintf(args_buffer, "{\"%s\": 100.0}", position_str.c_str());
		            gripper_command.args = std::string(args_buffer);
		            gripper_command_publisher.publish(gripper_command);
				}else{
					gripper_command.id = 65664;
		            gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
		            char args_buffer[50];
		            std::string position_str("position");
		            sprintf(args_buffer, "{\"%s\": 0.0}", position_str.c_str());
		            gripper_command.args = std::string(args_buffer);
		            gripper_command_publisher.publish(gripper_command);
				}
			}
				
			move_srv_success = false;
			//move_srv.request.target_pose=good_poses[qual_index][RandIndex];
			move_srv.request.target_pose=explorer.getAvailablePose(good_poses_index[qual_index][RandIndex]);
			std::cout << "send target pose to server: go to good_poses[" << qual_index << "][" << RandIndex << "]" << std::endl;
			while(!move_srv_success)
			{
				if (move_client.call(move_srv))
				{
					ROS_INFO("plan and execution succeed: %d", move_srv.response.succeed);
					move_srv_success = move_srv.response.succeed;
				}
				else
				{
					ROS_ERROR("Failed to call service move_to_target_pose...Retrying...");
				}
			}
		
			// verify that the scene and robot have translated into the correct qual state
			std::cout << "pause to wait for tf and msgs to be buffered ..." << std::endl;
			sleep(10.0);

			bool detect_service_succeed = false;
			while(!detect_service_succeed)
			{
				if (detect_client.call(detect_srv))
				{
					ROS_INFO("detection results: %d, %d, %d", detect_srv.response.bottle_hand, detect_srv.response.bottle_tabletop, detect_srv.response.hand_tabletop);
					detect_service_succeed = detect_srv.response.succeed;
				}
				else
				{
					ROS_ERROR("Failed to call service detect_touch...Retrying...");
					return 1;
				}
				r.sleep();
			}
			if(check_qual(detect_srv.response, qual_index+1))
			{
				printf("	transition q[%d]->q[%d]: complete", qual_index, qual_index+1);
				imitation_success[qual_index] = true;
				
				// record feature
				bool service_succeed = false;
				while(!service_succeed)
				{
					if (feature_client.call(feature_srv))
					{
						ROS_INFO("succeed: %d", feature_srv.response.succeed);
			            service_succeed = feature_srv.response.succeed;
					}
					else
					{
						ROS_ERROR("Failed to call service record_feature");
						return 1;
					}
					r.sleep();
				}
			}else {
				printf("	transition q[%d]->q[%d]: incomplete", qual_index, qual_index+1);
				break;
			}		
			
		}
		if(imitation_success[0] & imitation_success[1]) success_count++;
		count ++;
	}
	
	printf("imitation success rate: %f\n", (float)success_count/exp_num);
}
