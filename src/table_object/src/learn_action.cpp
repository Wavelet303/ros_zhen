/**
 * \file learn_action.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief estimate the qualitative state transition probability given a target pose, modeled as 6 dimension multivariate gaussian distribution
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include "table_object/detect_touch.h"
#include "baxter_moveit/move_to_target_pose.h"
#include "table_object/table_obj_exp.h"

#include <mlpack/core.hpp>
#include <mlpack/core/dists/gaussian_distribution.hpp>
#include <armadillo>

#define GRID_NUM 10

#define MIN_EXP_X -0.2
#define MAX_EXP_X 0.2

#define MIN_EXP_Y 0
#define MAX_EXP_Y 0.2

#define MIN_EXP_Z -0.05
#define MAX_EXP_Z 0.4

#define MIN_EXP_ROLL -3.14
#define MAX_EXP_ROLL 3.14

#define MIN_EXP_PITCH -1.5
#define MAX_EXP_PITCH 1.5

#define MIN_EXP_YAW -3
#define MAX_EXP_YAW 3

arma::mat observation;
arma::vec probability; 
int col_num = 0;
int transition_num = 1;
int exp_num = 1;

std::vector<std::vector<int> > human_demo;

bool check_valid(const arma::vec sample)
{
    if(sample[0] >= MIN_EXP_X & sample[0] <= MAX_EXP_X
     & sample[1] >= MIN_EXP_Y & sample[1] <= MAX_EXP_Y
     & sample[2] >= MIN_EXP_Z & sample[2] <= MAX_EXP_Z
     & sample[3] >= MIN_EXP_ROLL & sample[3] <= MAX_EXP_ROLL
     & sample[4] >= MIN_EXP_PITCH & sample[4] <= MAX_EXP_PITCH
     & sample[5] >= MIN_EXP_YAW & sample[5] <= MAX_EXP_YAW)
        return true;
    else
        return false;
}

void rpy_mod(arma::vec& sample)
{
    sample[3] = fmod(sample[3] + 3.14, 3.14) - 3.14;
    sample[4] = fmod(sample[4] + 3.14, 3.14) - 3.14;
    sample[5] = fmod(sample[5] + 3.14, 3.14) - 3.14;
}

bool check_qual(table_object::detect_touch::Response res)
{                                                           
    if(res.bottle_hand == (bool)human_demo[transition_num][0]       
        & res.bottle_tabletop == (bool)human_demo[transition_num][1]
        & res.hand_tabletop == (bool)human_demo[transition_num][2] )
        return true;
    else
        return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "learn_action");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    ros::ServiceClient detect_client = n.serviceClient<table_object::detect_touch>("detect_touch");
    table_object::detect_touch detect_srv;
    detect_srv.request.detect = true;
    
    ros::ServiceClient move_client = n.serviceClient<baxter_moveit::move_to_target_pose>("move_to_target_pose");
    baxter_moveit::move_to_target_pose move_srv;
    
    arma::arma_rng::set_seed_random();
    
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
    *  distribution initialization
    ***************************************/
    arma::vec initial_mean(6); // xyzrpy
    initial_mean << 0 << 0.1 << 0.075 << -3.14 << 1.047 << 1.57;
    arma::mat initial_cov(6,6);
    initial_cov.eye();
    initial_cov(0,0)=0.04;
    initial_cov(1,1)=0.02;
    initial_cov(2,2)=0.01;
    initial_cov(3,3)=0.3;
    initial_cov(4,4)=0.3;
    initial_cov(5,5)=0.3;
    mlpack::distribution::GaussianDistribution transition_prob(initial_mean, initial_cov);
    std::cout << "initial mean: "<< initial_mean.t() << std::endl;
    
    for(int grid_x=0; grid_x<GRID_NUM; grid_x++)
    {
        
    }
    
    /***************************************
    *  gather observation
    * 1. sample target pose
    ***************************************/
    arma::vec sample;
    while(1)
    {
       sample = transition_prob.Random();
       rpy_mod(sample);
       if(check_valid(sample))
           break;
           
    }
    std::cout << "sampled target pose: "<< sample.t() << std::endl;
    
    /***************************************
    *  gather observation
    * 2. plan and execute
    ***************************************/
    tf::Quaternion q;
    q.setRPY(sample(3), sample(4), sample(5));
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);

    geometry_msgs::Pose target_pose;
    target_pose.orientation = odom_quat;
    target_pose.position.x = sample(0);
    target_pose.position.y = sample(1);
    target_pose.position.z = sample(2);
    
    move_srv.request.target_pose=target_pose;
    
    std::cout << "send target pose to server " << std::endl;
    if (move_client.call(move_srv))
    {
        ROS_INFO("plan and execution succeed: %d", move_srv.response.succeed);
    }
    else
    {
        ROS_ERROR("Failed to call service move_to_target_pose");
        return 1;
    }
    
    /***************************************
    *  gather observation
    * 3. pause and apply event detectors
    ***************************************/
	bool transition_succeed = false;
    if(move_srv.response.succeed)
    {
    	std::cout << "pause to wait for tf and msgs to be buffered ..." << std::endl;
    	sleep(5.0);
    	
    	std::cout << "send request to detect service" << std::endl;
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
		        ROS_ERROR("Failed to call service detect_touch");
		        return 1;
		    }
		    r.sleep();
		}
		transition_succeed = check_qual(detect_srv.response);
    	std::cout << "transition succeed: " << transition_succeed << std::endl;
		
		/***************************************
		*  gather observation
		* 4. transition success probability of executed target pose (0 or 1)
		***************************************/
// 		observation.insert_cols(col_num, sample);
// 		//probability.insert_rows(col_num, (int)transition_succeed);
// 		arma::vec add_probability; 
// 		add_probability << (int)transition_succeed;
// 		probability.insert_rows(col_num, add_probability);
// 		
// 		col_num++;
    }
    
    
    /***************************************
    *  distribution estimation
    ***************************************/
    std::cout <<observation << std::endl;
    transition_prob.Estimate(observation,probability);
    std::cout << transition_prob.Mean() << std::endl;
    std::cout << transition_prob.Covariance() << std::endl;
    
}
