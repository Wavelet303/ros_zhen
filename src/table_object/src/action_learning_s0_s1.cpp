/**
 * \file action_learning_s0_s1.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief search all available actions and record the ones for completing 1st state transitions in observed behavior
 */
 
#include "table_object/table_obj_exp.h"
 
#include <ros/ros.h>
#include <tf/tf.h>

#include "table_object/detect_touch.h"
#include "table_object/palm_reflex_triggered.h"
#include "baxter_moveit/move_to_target_pose.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <time.h>
#include <cstdlib>

//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_msgs/AttachedCollisionObject.h>

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
	ros::init(argc, argv, "action_learning");
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
    
    ros::Publisher trigger_publisher = node.advertise<table_object::palm_reflex_triggered>("palm_reflex_triggered", 1);
    
//    ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
//    ros::Publisher attached_object_publisher = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    
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
    std::vector<std::vector<geometry_msgs::Pose> > good_poses;
    std::vector<std::vector<int> > good_poses_gripper_open; //open: 1; close: 0
    std::vector<std::vector<int> > good_poses_index;
    int qual_index;
	
	/***************************************
    *  search all available actions (home pose -> 12 different target poses)
    ***************************************/
    //int RandIndex = rand() % good_poses[qual_index-1].size(); //generates a random number between 0 and 3
	//move_srv.request.target_pose=good_poses[qual_index-1][RandIndex];
    for(qual_index=0; qual_index<1; qual_index++)
    {
    	ROS_INFO("learning action for qual state transition: %d -> %d", qual_index, qual_index+1);
		std::vector<geometry_msgs::Pose> cur_transition_good_poses;
		std::vector<int> cur_transition_good_poses_gripper_open;
		std::vector<int> cur_good_poses_index;
		for(int gripper_open=0; gripper_open<=1; gripper_open++)
		{
			for(int available_pose_index=0; available_pose_index < 7; available_pose_index++)
			{
			
				// close or open gripper 
				if(gripper_open)
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
			
				// get to home pose first
				bool move_srv_success = false;
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
				
				
				sleep(2.0);
		
				/*if(qual_index == 1) // hand and bottle should be touching
				{
					move_srv_success = false;
					// random choose one 
					srand ( time(NULL) ); //initialize the random seed
		  			int RandIndex = rand() % good_poses[qual_index-1].size(); //generates a random number between 0 and 3
		  			
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
		  			
					move_srv.request.target_pose=good_poses[qual_index-1][RandIndex];
					std::cout << "send target pose to server: go to good_poses[" << qual_index-1 << "][" << RandIndex << "]" << std::endl;
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
				}*/
		
				// verify that the scene and robot are in the correct qual state
				std::cout << "pause to wait for tf and msgs to be buffered ..." << std::endl;
				sleep(10.0);
		
				std::cout << "verify correct current qual state: send request to detect service" << std::endl;
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
				if(check_qual(detect_srv.response, qual_index))
				{
					std::cout << "current qual state is satisfied, start learning..." << std::endl;
					
					// attach bottle to robot gripper if needed
					/*if(qual_index==1 & available_pose_index==1)
					{
						// moved to palm_reflex_server.cpp
					}*/
				}else {
					std::cout << "current qual state is not satisfied..." << std::endl;
					exit(1);
				}		
		
				// close or open gripper: again here since palm reflex might be triggered previously
				/*if(gripper_open)
				{
					gripper_command.id = 65664;
                    gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
                    char args_buffer[50];
                    std::string position_str("position");
                    sprintf(args_buffer, "{\"%s\": 100.0}", position_str.c_str());
                    gripper_command.args = std::string(args_buffer);
                    gripper_command_publisher.publish(gripper_command);
                    
                    table_object::palm_reflex_triggered signal;
                    signal.palm_reflex_triggered = false;
                    trigger_publisher.publish(signal);                    
				}else{
					gripper_command.id = 65664;
                    gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
                    char args_buffer[50];
                    std::string position_str("position");
                    sprintf(args_buffer, "{\"%s\": 0.0}", position_str.c_str());
                    gripper_command.args = std::string(args_buffer);
                    gripper_command_publisher.publish(gripper_command);
				}
				sleep(2.0);*/
				
				// try out available_pose_index pose
				move_srv_success = false;
				move_srv.request.target_pose=explorer.getAvailablePose(available_pose_index);
				std::cout << "send target pose to server: go to " << available_pose_index << "th avaialbe pose" << std::endl;
				//while(!move_srv_success)
				//{
					if (move_client.call(move_srv))
					{
						ROS_INFO("plan and execution succeed: %d", move_srv.response.succeed);
						move_srv_success = move_srv.response.succeed;
					}
					else
					{
						ROS_ERROR("Failed to call service move_to_target_pose...Retrying...");
					}
				//}
		
				if(move_srv_success)
				{
					// pause and apply event detectors
					std::cout << "pause to wait for tf and msgs to be buffered ..." << std::endl;
					sleep(10.0);
		
					std::cout << "send request to detect service" << std::endl;
					detect_service_succeed = false;
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
						std::cout << "good pose! recording..." << std::endl;
						cur_transition_good_poses.push_back(explorer.getAvailablePose(available_pose_index));
						cur_transition_good_poses_gripper_open.push_back(gripper_open);
						cur_good_poses_index.push_back(available_pose_index);
					}else{
						std::cout << "state transition is not completed" << std::endl;
					}
				}
				
				
				// open gripper
				gripper_command.id = 65664;
		        gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
		        char args_buffer[50];
		        std::string position_str("position");
		        sprintf(args_buffer, "{\"%s\": 100.0}", position_str.c_str());
		        gripper_command.args = std::string(args_buffer);
		        gripper_command_publisher.publish(gripper_command);
				
			}
		}
		
		good_poses.push_back(cur_transition_good_poses);
		good_poses_gripper_open.push_back(cur_transition_good_poses_gripper_open);
		good_poses_index.push_back(cur_good_poses_index);
	}
	
	/***************************************
    *  printout learning results
    ***************************************/
    std::cout << "learning results: " << std::endl;
    for(int i=0; i<good_poses.size(); i++)
    {
    	printf("	transition q[%d]->q[%d]: good poses are", i, i+1);
    	for(int j=0; j<good_poses[i].size(); j++)
    	{
    		printf("		available_pose[%d]: %f, %f, %f, gripper open %d\n", good_poses_index[i][j], good_poses[i][j].position.x, good_poses[i][j].position.y, 
    														good_poses[i][j].position.z, good_poses_gripper_open[i][j]);
		}
    	
    }
    
}
