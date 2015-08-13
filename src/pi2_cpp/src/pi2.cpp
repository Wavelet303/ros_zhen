/**
 * \file        pi2.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <math.h>
#include <fstream>
#include <cstdio>
#include <random>
#include <algorithm>
#include <vector>
#include <ctime>

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {
	
	// initialize original index locations
	std::vector<size_t> idx(v.size());
	for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;
	
	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
		 [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
	
	return idx;
}

PI2::PI2(int n_dmps, int n_basis,  bool update_goal)
{
	_n_dmps = n_dmps;
	_n_basis = n_basis;
	
	for(int i=0; i<_n_dmps; i++)
	{
		DMP dmp(_n_basis);
		_dmps.push_back(dmp);
	}
	
	_HeavyFmt = Eigen::IOFormat(Eigen::FullPrecision);
	
	//setup ROS related variables
	_ROS_initialized = false;
	_ee_record_count = 0;	
	
	_dt = 0.05;
	
	_update_goal = update_goal;
}

void PI2::readProtocol(std::string protocol_name)
{
	std::ifstream infile(protocol_name.c_str());
	std::string line;
	
	while(std::getline(infile, line))
	{
		if(line.length()==0 || line[0]=='%')
			continue;
		
		std::istringstream iss(line);
		
		// read in start states
		for(int i=0; i<_n_dmps;i++)
		{
			double start;
			if(!(iss>>start))
			{
				std::cout << "error reading starting positions\n";
				exit(1);
			}
			_protocol.start.push_back(start);
		}
		
		// read in goal states
		for(int i=0; i<_n_dmps; i++)
		{
			double goal;
			if(!(iss>>goal))
			{
				std::cout << "error reading goal positions\n";
				exit(1);
			}
			_protocol.goal.push_back(goal);
		}
		
		// read in duration, std, repetitions, cost_function, update, basis_noise, n_reuse
		if(!(iss >>_protocol.duration >>_protocol.stdv >>_protocol.reps
			>>_protocol.cost_function >>_protocol.updates >>_protocol.basis_noise
			>>_protocol.n_reuse))
		{
			std::cout << "error reading porotocol file\n";
            exit(1);
		}
	}
	std::cout << _protocol.cost_function << "\n";
}

void PI2::initializeW(char* dmp_folder_name)
{	
	_dmp_folder_name = dmp_folder_name;
	
	Eigen::MatrixXd read_T;
	for(int i=0; i<_n_dmps; i++)
	{
		char buffer [100];
		sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/dmp%d_smooth.txt", dmp_folder_name, i);
		read_T= _dmps[i].readMatrix(buffer, false);
		// 		std::cout << "read file T = " << read_T << std::endl; 
		
		Eigen::Map<Eigen::VectorXd> T(read_T.data(),read_T.cols()*read_T.rows(),1);
		_dmps[i].batch_fit(0.01*T.size(), 0.01, T); //0.01s corresponds to 100Hz at which endpoint state is published/received
	}
	
	//if _n_dmps == 8, i.e., ee_position(3) + ee_orientation(4) + delta_close (1)
	if(_n_dmps==8)
	{
		Eigen::MatrixXd close_gripper_delta_function(1,read_T.cols());
		int jump_index = 10;
		for(int i=0; i<jump_index; i++)
		{
			close_gripper_delta_function(i)=0;
		}
		for(int i=jump_index; i<read_T.cols(); i++)
		{
			close_gripper_delta_function(i)=0;
		}
	}
}

void PI2::loadLearnedW(char* dmp_folder_name)
{
	_dmp_folder_name = dmp_folder_name;
	for(int i=0; i<_n_dmps; i++)
	{
		_dmps[i].loadWFromFile(dmp_folder_name, i);
	}
}

void PI2::writeGoalToFile(char* dmp_folder_name)
{
	_dmp_folder_name = dmp_folder_name;
	char buffer [100];
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/goal_learned.txt", dmp_folder_name);
	std::ofstream fout;
	fout.open(buffer);
	
	for(int i=0; i<(int)_protocol.goal.size(); i++)
	{
		fout << "   " << _protocol.goal[i];
	}
	fout.close();
}

void PI2::loadLearnedGoal(char* dmp_folder_name)
{
	_dmp_folder_name = dmp_folder_name;
	char buffer [100];
	sprintf (buffer, "/home/zengzhen/Desktop/kinesthetic_teaching/%s/goal_learned.txt", dmp_folder_name);
	
	DMP dummy(10);
	Eigen::MatrixXd temp_matrix = dummy.readMatrix(buffer, false);
	Eigen::VectorXd temp_vector = temp_matrix.transpose();
	std::vector<double> learned_goal(temp_vector.data(), temp_vector.data()+temp_vector.size());
	_protocol.goal = learned_goal;
}


void PI2::setROSNodeHandle(ros::NodeHandle& n)
{
	_ROS_initialized = true;
	
	_n = n;
	_publisher = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1000);
	_gripper_publiser = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
	_client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
	_subscriber = _n.subscribe("/robot/limb/left/endpoint_state", 100, &PI2::eeStateCallback, this);
}


void PI2::runProtocol()
{
	double dt = _dt;
	int n = 1.5*(int)round(_protocol.duration / dt);
	
	PI2Data D(n, _n_dmps, _n_basis, _protocol.duration, dt, _protocol.goal);
	
	PI2Data D_eval = D;
	
	//one data structure for each repetition
	std::vector<PI2Data> D_series;
	for(int i=0; i<_protocol.reps; i++)
	{
		PI2Data copyD(n, _n_dmps, _n_basis, _protocol.duration, dt, _protocol.goal);
		D_series.push_back(copyD);
	}
	std::vector<PI2Data> D_eval_series;
	D_eval_series.push_back(D_eval);
	
	//used to store the cost as learning happens
	Eigen::MatrixXd T;
	T.setZero(_protocol.updates+1, 2);
	
	std::string epsilon_file("/home/zengzhen/Desktop/epsilon.txt");
	std::ifstream infile(epsilon_file.c_str());
	
	if(_update_goal)
	{
		std::cout << "initial goal: ";
		for(int i=0; i<(int)_protocol.goal.size(); i++)
			std::cout << _protocol.goal[i] << " ";
		std::cout<< std::endl;
	}
	
	PI2Protocol p_eval;
	for(int i=0; i<_protocol.updates;i++)
	{
		p_eval = _protocol;
		p_eval.reps = 1;
		p_eval.stdv = 0;
		p_eval.n_reuse = 0;
		
		run_rollouts(D_eval_series, p_eval,1, &infile);
		
		//print out commanded trajectory
		std::cout << D_eval_series[0].dmp[0].y.transpose() << std::endl; //position.x trajectory
		
		//compute all costs in batch form, as this is faster in matlab
		Eigen::MatrixXd R_eval = cost(D_eval_series); 
		std::cout << std::endl;
		
		/**********************************************************************
		* run again to see how much the cost varies given the exact same DMP
		***********************************************************************/
// 		run_rollouts(D_eval_series, p_eval,1, &infile);
// 		
// 		//print out commanded trajectory
// 		std::cout << D_eval_series[0].dmp[0].y.transpose() << std::endl; //position.x trajectory
// 		
// 		//compute all costs in batch form, as this is faster in matlab
// 		R_eval = cost(D_eval_series); 
// 		std::cout << std::endl;
// 		exit(1);
		
		if(i==0)
		{
			T(0,0) = 1;
			T(0,1) = R_eval.colwise().sum()(0);
		}else{
			T(i,0) = i*(_protocol.reps-_protocol.n_reuse)+_protocol.n_reuse+1;
			T(i,1) = R_eval.colwise().sum()(0);
		}
		
		if(i % 10 == 0)
		{
			std::cout << "update " << i << ": cost = " << T(i,1) << "\n";
		}
		
		//run learning roll-outs with a noise annealing multiplier
		double noise_mult = (double)(_protocol.updates-i-1)/(double)_protocol.updates;
		noise_mult = std::max(noise_mult, 0.1);
		run_rollouts(D_series, _protocol, noise_mult, &infile);
		
		//compute all costs in batch form, as this is faster in Matlab
		Eigen::MatrixXd R = cost(D_series);
		std::cout << std::endl;
		
		//perform the PI2 update
		updatePI2(D_series, R);
		
		//reuse of roll-outs: the n_reuse best trials and re-evaluate them the next update in
		//the spirit of importance sampling
		if((i>0) & (_protocol.n_reuse>0))
		{
			Eigen::VectorXd sum_R = R.colwise().sum();
			std::vector<double> sum_R_vector(sum_R.data(), sum_R.data()+sum_R.size());
			std::vector<size_t> inds;
			inds = sort_indexes<double>(sum_R_vector);
			for(int j=0; j<_protocol.n_reuse; j++)
			{
				PI2Data Dtemp = D_series[j];
				D_series[j] = D_series[inds[j]];
				D_series[inds[j]] = Dtemp;
			}
		}
	}
	
	//perform the final noiseless evaluation to get the final cost
	run_rollouts(D_eval_series, p_eval, 1);
	Eigen::MatrixXd R_eval = cost(D_eval_series);
	T(_protocol.updates,0) = _protocol.updates*(_protocol.reps-_protocol.n_reuse)+_protocol.n_reuse+1;
	T(_protocol.updates,1) = R_eval.colwise().sum()(0);
	
	//print out commanded trajectory
	std::cout << std::endl;
	std::cout << D_eval_series[0].dmp[0].y.transpose() << std::endl; //position.x trajectory
	
	std::cout << "cost trace = " << T.format(_HeavyFmt) << std::endl;
	
	for(int i=0; i<_n_dmps; i++)
	{
// 		std::cout << _dmps[i].getW().transpose() << std::endl;
		_dmps[i].writeWToFile(_dmp_folder_name, i);
	}
	
	if(_update_goal)
	{
		std::cout << "learned goal: ";
		for(int i=0; i<(int)_protocol.goal.size(); i++)
			std::cout << _protocol.goal[i] << " ";
		std::cout<< std::endl;
		writeGoalToFile(_dmp_folder_name);
	}
}

void PI2::runProtocolLearnedW()
{
	double dt = _dt;
	int n = 1.5*(int)round(_protocol.duration / dt);
	
	PI2Data D(n, _n_dmps, _n_basis, _protocol.duration, dt, _protocol.goal);
	
	PI2Data D_eval = D;
	std::vector<PI2Data> D_eval_series;
	D_eval_series.push_back(D_eval);
	
	PI2Protocol p_eval;
	p_eval = _protocol;
	p_eval.reps = 1;
	p_eval.stdv = 0;
	p_eval.n_reuse = 0;
		
	run_rollouts(D_eval_series, p_eval,1);
}


void PI2::run_rollouts(std::vector<PI2Data>& D, PI2Protocol p, double noise_mult, std::ifstream* infile)
{
	std::string line;
	
	double dt = D[0].dt;
	
	//run roll-outs
	int start = p.n_reuse;
	if(D[0].dmp[0].psi(0,0)==0) //indicates very first batch of run_rollouts
		start = 0;
	
	double epsilon_g[_n_dmps];
	// generate desired trajectory
	for(int k = start; k<p.reps; k++ )
	{
		//reset the DMP
		double goal_pos_exploration = 0.03*noise_mult;
		double goal_ori_exploration = 0.07*noise_mult;
		for(int j=0; j<_n_dmps; j++)
		{
			_dmps[j].reset_state(p.start[j]);
			
			epsilon_g[j]=0.0;
			if((p.stdv!=0) & _update_goal)
			{
				if(j<3)
					epsilon_g[j] += _distribution(_generator)*goal_pos_exploration;
				else
					epsilon_g[j] += _distribution(_generator)*goal_ori_exploration;
			}
			_dmps[j].set_goal(p.goal[j]+epsilon_g[j], 1);
		}
		
		//integrate through the duration
		for(int n=0; n<D[k].dmp[0].y.size(); n++)
		{
			double std_eps = p.stdv * noise_mult;
			
			for(int j=0; j<_n_dmps; j++)
			{
// 				printf("loop k=%d, n=%d, j=%d\n", k, n, j);
				
				Eigen::VectorXd epsilon;
				epsilon.setZero(_n_basis);
				
				//generate noise_mult
				if(!p.basis_noise) //this case adds noise at every time step
				{
					for(int epsilon_ind=0; epsilon_ind<10; epsilon_ind++)
						epsilon(epsilon_ind) = _distribution(_generator)*std_eps;
					
				}else{ //this case only adds noise for the most active basis function,
					   //and noise does not change during hte activity of the basis function
					if(n==0)
					{
						epsilon(0)=_distribution(_generator)*std_eps;
						epsilon.block(1,0, _n_basis-1,1) = epsilon.block(1,0, _n_basis-1,1)*0;
						
						//DEBUG load epsilon from file generated by Matlab
// 						if(infile!=NULL)
// 						{
// 							std::getline(*infile, line);
// 							std::istringstream iss(line);
// 							double rnd_epsilon;
// 							if(!(iss>>rnd_epsilon))
// 							{
// 								printf("loop k=%d, n=%d, j=%d: ", k, n, j);
// 								std::cout << "error reading epsilon file\n";
// 								exit(1);
// 							}
// 							epsilon(0) = rnd_epsilon;
// // 							printf("n=%d: %.63f\n", n, epsilon(0));
// 						}
					}else{
						//what is the max activated basis function from the previous time step?
						Eigen::ArrayXd psi_row_array = D[k].dmp[j].psi.row(n-1);
						std::vector<double> psi_row(psi_row_array.data(), psi_row_array.data()+psi_row_array.rows()*psi_row_array.cols());
						int ind_basis = std::distance(psi_row.begin(), std::max_element(psi_row.begin(), psi_row.end()));
						
						//what was the noise vector from the previous time step?
						Eigen::ArrayXd epsilon_prev = D[k].dmp[j].theta_eps.row(n-1) - _dmps[j].getW().transpose();
						Eigen::ArrayXd epsilon_prev_abs = epsilon_prev.abs();
						//...and find the index of the basis function to which we added the noise previously
						std::vector<double> epsilon_prev_row(epsilon_prev_abs.data(), epsilon_prev_abs.data()+epsilon_prev_abs.rows()*epsilon_prev_abs.cols());
						int ind_eps = std::distance(epsilon_prev_row.begin(), std::max_element(epsilon_prev_row.begin(), epsilon_prev_row.end()));
						
						//only add new noise if max basis function index changed
						if(ind_eps != ind_basis)
						{
							epsilon.setZero(_n_basis);
							epsilon(ind_basis) = _distribution(_generator)*std_eps;
							
							//DEBUG load epsilon from file generated by Matlab
// 							if(infile!=NULL)
// 							{
// 								std::getline(*infile, line);
// 								std::istringstream iss(line);
// 								double rnd_epsilon;
// 								if(!(iss>>rnd_epsilon))
// 								{
// 									printf("loop k=%d, n=%d, j=%d: ", k, n, j);
// 									std::cout << "error reading epsilon file\n";
// 									exit(1);
// 								}
// 								epsilon(ind_basis) = rnd_epsilon;
// // 								printf("n=%d: %.63f\n", n, epsilon(ind_basis));
// 							}
						}else
							epsilon = epsilon_prev.transpose();
					}
				}
				
				//after duration/dt no noise is added anymore
				if(n >= (int)round(p.duration/dt))
					epsilon.setZero(_n_basis);
				
				//integrate DMP
				std::vector<Eigen::VectorXd> dmp_stats;
				dmp_stats = _dmps[j].run(p.duration, dt, 0, 0, 1, 1, epsilon);
				
				//store D
				D[k].dmp[j].y(n) = dmp_stats[0](0); //y
				D[k].dmp[j].yd(n) = dmp_stats[1][0]; //yd
				D[k].dmp[j].ydd(n) = dmp_stats[2][0]; //ydd
				D[k].dmp[j].bases.row(n) = dmp_stats[3].transpose(); //b 
				D[k].dmp[j].theta_eps.row(n) = (_dmps[j].getW() + epsilon).transpose();
				D[k].dmp[j].psi.row(n) = _dmps[j].getPsi().transpose();
				D[k].dmp[j].g_eps = epsilon_g[j];
			}
		}
		
		if(_n_dmps>=7)
		{
			//run generated desired trajectory on Baxter
			run_baxter(D, k);
		}
	}
}

void PI2::run_baxter(std::vector<PI2Data>& D, int trial_index)
{
	//only execute the roll-outs when ros node has been set up and publisher & subscriber are set
	if(_ROS_initialized)
	{	
		_ee_current_time.clear();
		_average_ee_pose.clear();
		_average_ee_pose_time_stamp.clear();
		_ee_record_count = 0;
		for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
		{
			geometry_msgs::Pose pose;
			pose.position.x = 0;
			pose.position.y = 0;
			pose.position.z = 0;
			pose.orientation.x = 0;
			pose.orientation.y = 0;
			pose.orientation.z = 0;
			pose.orientation.w = 0;
			_average_ee_pose.push_back(pose);
			
			_average_ee_pose_time_stamp.push_back(0);
		}
		
		baxter_core_msgs::JointCommand msg;
		msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
		
		_srv.request.pose_stamp.clear();
		
		//command previously generated trajectory to Baxter
		for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
		{			
			//construct left limb endpoint msg (all DOFs at the same time step)
			geometry_msgs::PoseStamped ee_pose;
			ee_pose.header.stamp = ros::Time::now();
			ee_pose.header.frame_id = "base";
// 			ee_pose.pose.position.x = 0.3+0.001*n;
// 			ee_pose.pose.position.y = 0.6;
// 			ee_pose.pose.position.z = 0.05;
// 			ee_pose.pose.orientation.x = -0.018;
// 			ee_pose.pose.orientation.y = 0.994;
// 			ee_pose.pose.orientation.z = -0.003;
// 			ee_pose.pose.orientation.w = 0.111;
			ee_pose.pose.position.x = D[trial_index].dmp[0].y(n);
			ee_pose.pose.position.y = D[trial_index].dmp[1].y(n);
			ee_pose.pose.position.z = D[trial_index].dmp[2].y(n);
			ee_pose.pose.orientation.x = D[trial_index].dmp[3].y(n);
			ee_pose.pose.orientation.y = D[trial_index].dmp[4].y(n);
			ee_pose.pose.orientation.z = D[trial_index].dmp[5].y(n);
			ee_pose.pose.orientation.w = D[trial_index].dmp[6].y(n);
			
			//call IKService to get desired joint positions
			_srv.request.pose_stamp.push_back(ee_pose);
		}
		
		if (_client.call(_srv))
		{
			bool all_valid = true;
			for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
			{
				if(!_srv.response.isValid[n])
				{
					all_valid = false;
					break;
				}
			}
			
			if(all_valid)
			{
				ROS_INFO("SUCCESS - All Valid Joint Solution Found");
// 					for(int i=0; i<(int)_srv.response.joints[0].name.size(); i++)
// 						std::cout << _srv.response.joints[0].name[i] << " ";
// 					std::cout << std::endl;
// 					for(int i=0; i<(int)_srv.response.joints[0].position.size(); i++)
// 						std::cout << _srv.response.joints[0].position[i] << " ";
// 					std::cout << std::endl;
			}else
				ROS_INFO("INVALID POSE - Not all Valid Joint Solution Found.");
		}
		else
		{
			ROS_ERROR("Failed to call service add_two_ints");
			exit(1);
		}
			
		ros::Time begin = ros::Time::now();
		double time_offset = 2; //5 is safe
		
		for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
		{
			time_offset = time_offset + D[trial_index].dt;
			
			//construct left limb joint positions msg
			msg.names = _srv.response.joints[n].name;
			msg.names.push_back("left_gripper");
			msg.command = _srv.response.joints[n].position;
			msg.command.push_back(100.0);
			
// 			std::cout << "**********************run baxter*********************************" << std::endl;
// 			std::cout << "current time from starting time = " << (ros::Time::now()-begin).toSec() << std::endl;
// 			std::cout << "time_offset = " << time_offset << std::endl;
			//command Baxter to the desired joint position
			_ee_current_time.push_back(begin.toSec()+time_offset);
			while((ros::Time::now()-begin).toSec()<time_offset)
			{
				_publisher.publish(msg);
				ros::AsyncSpinner spinner(4); // Use 4 threads
				spinner.start();
				spinner.stop();
			}
			
// 			if(n==10)
// 			{
// // 				while((ros::Time::now()-begin).toSec()<time_offset+0.5) //don't need this beyong this test(since n is double the length duration/dt)
// // 					_publisher.publish(msg);
// 				
// 				for(int nn=0; nn<11; nn++)
// 				{
// 					printf("position: %f, %f, %f\n", _average_ee_pose[nn].position.x, _average_ee_pose[nn].position.y, _average_ee_pose[nn].position.z);
// 					printf("orientation: %f, %f, %f, %f\n", _average_ee_pose[nn].orientation.x, _average_ee_pose[nn].orientation.y, _average_ee_pose[nn].orientation.z, _average_ee_pose[nn].orientation.w);
// 				}
// 				exit(1);
// 			}
		}
		
		if(_ee_record_count != (int)_average_ee_pose.size())
		{
			std::cout << "Lost some poses at some requested time stamp\n";
			exit(1);
		}
		
// 		for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
// 		{
// 			printf("position: %f, %f, %f\n", _average_ee_pose[n].position.x, _average_ee_pose[n].position.y, _average_ee_pose[n].position.z);
// 			printf("orientation: %f, %f, %f, %f\n", _average_ee_pose[n].orientation.x, _average_ee_pose[n].orientation.y, _average_ee_pose[n].orientation.z, _average_ee_pose[n].orientation.w);
// 		}


		int steps = D[trial_index].dmp[0].y.size();		
		if(_update_goal)
		{
			//close gripper - grasp
			baxter_core_msgs::EndEffectorCommand gripper_command;
			gripper_command.id = 65664;
			gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
			_gripper_publiser.publish(gripper_command);
			ros::Duration(1).sleep();
			
			//increase z value - lift
			geometry_msgs::PoseStamped ee_pose;
			ee_pose.header.stamp = ros::Time::now();
			ee_pose.header.frame_id = "base";
			ee_pose.pose.position.x = D[trial_index].dmp[0].y(steps-1);
			ee_pose.pose.position.y = D[trial_index].dmp[1].y(steps-1);
			ee_pose.pose.position.z = D[trial_index].dmp[2].y(steps-1)+0.2;
			ee_pose.pose.orientation.x = D[trial_index].dmp[3].y(steps-1);
			ee_pose.pose.orientation.y = D[trial_index].dmp[4].y(steps-1);
			ee_pose.pose.orientation.z = D[trial_index].dmp[5].y(steps-1);
			ee_pose.pose.orientation.w = D[trial_index].dmp[6].y(steps-1);
			_srv.request.pose_stamp.clear();
			_srv.request.pose_stamp.push_back(ee_pose);
			if (_client.call(_srv))
			{	
				if(_srv.response.isValid[0])
				{
					ROS_INFO("SUCCESS - Valid Joint Solution Found for Lifting");
				}else
					ROS_INFO("INVALID POSE - Not all Valid Joint Solution Found.");
			}
			else
			{
				ROS_ERROR("Failed to call service add_two_ints");
				exit(1);
			}
			msg.names = _srv.response.joints[0].name;
			msg.names.push_back("left_gripper");
			msg.command = _srv.response.joints[0].position;
			msg.command.push_back(0.0);
			begin = ros::Time::now();
			while((ros::Time::now()-begin).toSec()<2)
			{
				_publisher.publish(msg);
			}
			ros::Duration(1).sleep();
			
			//open gripper - release
// 			gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
// 			_gripper_publiser.publish(gripper_command);
		}

		//ask for cost from user
		double terminal_cost = 0.0;
		if(_update_goal)
		{
			std::cout << "Please enter terminal cost: ";
			std::cin >> terminal_cost;
		}
		D[trial_index].user_input_cost = terminal_cost;

		//store D[trial_index]: q
		for(int n=0; n<D[trial_index].dmp[0].y.size(); n++)
		{
			D[trial_index].q(n, 0) = _average_ee_pose[n].position.x;
			D[trial_index].q(n, 1) = _average_ee_pose[n].position.y;
			D[trial_index].q(n, 2) = _average_ee_pose[n].position.z;
			
			D[trial_index].q(n, 3) = _average_ee_pose[n].orientation.x;
			D[trial_index].q(n, 4) = _average_ee_pose[n].orientation.y;
			D[trial_index].q(n, 5) = _average_ee_pose[n].orientation.z;
			D[trial_index].q(n, 6) = _average_ee_pose[n].orientation.w;
		}
		
		//store D[trial_index]: qd
		for(int i=0; i<7; i++)
		{
			D[trial_index].qd.col(i).block(0,0,steps-1,1) = D[trial_index].q.col(i).block(1,0,steps-1,1);
			D[trial_index].qd.col(i) = D[trial_index].qd.col(i) - D[trial_index].q.col(i);
			D[trial_index].qd(steps-1,i) = 0;
// 			D[trial_index].qd.col(i) = D[trial_index].qd.col(i)/D[trial_index].dt;
			
			for(int j=0; j<steps-1; j++)
				D[trial_index].qd(j,i) = D[trial_index].qd(j,i)/(_average_ee_pose_time_stamp[j+1]-_average_ee_pose_time_stamp[j]);
		}
		
		//store D[trial_index]: qdd
		for(int i=0; i<7; i++)
		{
			D[trial_index].qdd.col(i).block(0,0,steps-1,1) = D[trial_index].qd.col(i).block(1,0,steps-1,1);
			D[trial_index].qdd.col(i) = D[trial_index].qdd.col(i) - D[trial_index].qd.col(i);
			D[trial_index].qdd(steps-1,i) = 0;
// 			D[trial_index].qdd.col(i) = D[trial_index].qdd.col(i)/D[trial_index].dt;
			
			for(int j=0; j<steps-1; j++)
				D[trial_index].qdd(j,i) = D[trial_index].qdd(j,i)/(_average_ee_pose_time_stamp[j+1]-_average_ee_pose_time_stamp[j]);
		}
		
// 		std::cout << "y = " << D[trial_index].dmp[0].y.transpose() << std::endl;
// 		std::cout << "\nq = " << D[trial_index].q.col(0).transpose() << std::endl;
// 		std::cout << "\nqd = " << D[trial_index].qd.col(0).transpose() << std::endl;
// 		std::cout << "\nqdd = " << D[trial_index].qdd.col(0).transpose() << std::endl;
// 		exit(1);
	}
}


Eigen::MatrixXd PI2::cost(std::vector< PI2Data > D)
{	
	int n_reps = D.size();
	int n = D[0].dmp[0].y.size(); //the length of a trajectory in time steps
	int n_real = (int)round(D[0].duration/D[0].dt); // the duration of the core trajectory in time steps -- everything beyond this time belongs to the terminal cost
	
	Eigen::MatrixXd R;
	R.setZero(n, n_reps);
	
	Eigen::VectorXd r_ydd;
	r_ydd.setZero(n_real);
	
	//compute cost
	for(int k=0; k<n_reps; k++)
	{
		Eigen::VectorXd r, rt;
		r.setZero(n_real);
		rt.setZero(n-n_real);
		
		for(int i=0; i<_n_dmps; i++)
		{
			//cost during trajectory
// 			if(_n_dmps<7)
// 			{
				//implement the "acc2_exp.m" cost function + terminal cost
				Eigen::VectorXd ydd;
				ydd = D[k].dmp[i].ydd.block(0,0,n_real,1);
// 				ydd = ydd.cwiseAbs();
// 				ydd = ydd*(-0.01);
// 				ydd = ydd.array().exp();
// 				Eigen::VectorXd ones_col;
// 				ones_col.setOnes(ydd.size(),1);
// 				r = r + (ones_col-ydd)/n_real;
				if(n_reps==1)
				{
					ydd = ydd.cwiseProduct(ydd);
					r_ydd = r_ydd + ydd;
				}
// 			}else{
				//cost = acceleration^2 sum over all DOFs
				Eigen::VectorXd qdd;
				qdd = D[k].qdd.block(0,i,n_real,1);
// 				qdd = qdd.cwiseAbs();
// 				qdd = qdd*(-0.01);
// 				qdd = qdd.array().exp();
// 				Eigen::VectorXd ones_col;
// 				ones_col.setOnes(qdd.size(),1);
// 				r = r + (ones_col-qdd)/n_real;
				qdd = qdd.cwiseProduct(qdd);
				r = r + qdd;
				
// 				std::cout << "ydd = " << D[k].dmp[i].ydd.block(0,0,n_real,1).transpose() << std::endl;
// 				std::cout << "\nq = " << D[k].qdd.block(0,i,n_real,1).transpose() << std::endl;
// 				std::cout << "sum(ydd.^2) = " << ydd.sum() << std::endl;
// 				std::cout << "sum(qdd.^2) = " << qdd.sum() << std::endl;
				
// 			}
		}
		
		if(_update_goal)
			r = r*0.0001;
		
		//terminal cost: task specific
		rt(rt.size()-1) = D[k].user_input_cost;
		
		Eigen::VectorXd rrt;
		rrt.setZero(r.size()+rt.size());
		rrt.block(0,0,r.size(),1) = r;
		rrt.block(r.size(),0,rt.size(),1) = rt;
		R.col(k) = rrt;
	}
	
	if(n_reps==1)
		std::cout << "sum(r_ydd) = " << r_ydd.sum() << std::endl;
	
	std::cout << "cost function: sum(R) = " << R.colwise().sum() << " ";
	
	return R;
}

void PI2::updatePI2(std::vector< PI2Data > D, Eigen::MatrixXd R)
{
	//compute the parameter update with PI2
	int n = R.rows();
	int n_reps = R.cols();
	
	//compute the accumulate cost
	Eigen::MatrixXd rot90R, rot180R;
	Eigen::MatrixXd S;
	rot90R = R.transpose().colwise().reverse(); // rot90(R)
	rot180R = rot90R.transpose().colwise().reverse(); // rot90(rot90(R))
	
	for(int i=0; i<rot180R.cols(); i++)
	{
		Eigen::VectorXd S_col = rot180R.col(i);
		std::vector<double> S_col_i(S_col.data(), S_col.data()+S_col.size());
		std::vector<double> Sum = S_col_i;
		std::partial_sum(S_col_i.begin(), S_col_i.end(), Sum.begin());
		Eigen::Map<Eigen::VectorXd> temp(&Sum[0], Sum.size());
		rot180R.col(i) = temp;
	}
	rot90R = rot180R.transpose().colwise().reverse();
	S = rot90R.transpose().colwise().reverse();	
	
	//compute the exponentiated cost with the special trick to automatically adjust the lambda scaling paramter
	Eigen::VectorXd maxS, minS;
	maxS.setZero(S.rows());
	minS.setZero(S.rows());
	for(int i=0; i<S.rows(); i++)
	{
		maxS(i) = S.row(i).maxCoeff();
		minS(i) = S.row(i).minCoeff();
	}
	
	double h = 10; //this is the scaling parameters in side of the exp() function (see README.pdf in PI2 doc)
	Eigen::VectorXd row_ones;
	row_ones.setOnes(n_reps);
	Eigen::MatrixXd denominator = (maxS-minS)*row_ones.transpose();
	Eigen::MatrixXd matrix_small;
	matrix_small.setConstant(denominator.rows(), denominator.cols(), 1.e-10);
	denominator = denominator + matrix_small;
	Eigen::MatrixXd nominator = -h*(S - minS*row_ones.transpose());
	Eigen::MatrixXd expS = nominator.cwiseQuotient(denominator);
	expS = expS.array().exp();
	
// 	std::cout << expS.format(_HeavyFmt) << std::endl; exit(1);
	
	//the probability of a trajectory
	denominator = expS.rowwise().sum();
	denominator = denominator*row_ones.transpose();
	Eigen::MatrixXd P = expS.cwiseQuotient(denominator);
    
	//compute the projected noise term. It is computationally more efficient to break this operation into inner product terms
	std::vector<std::vector<Eigen::MatrixXd>> PMeps;
	for(int i=0; i<_n_dmps; i++)
	{
		std::vector<Eigen::MatrixXd> temp2;
		for(int j=0; j<n_reps; j++)
		{
			Eigen::MatrixXd temp1;
			temp1.setZero(n, _n_basis);
			temp2.push_back(temp1);
		}
		PMeps.push_back(temp2);
	}
	
	for(int j=0; j<_n_dmps; j++)
	{
		for(int k=0; k<n_reps; k++)
		{			
			//compute g'*eps in vector form
			Eigen::MatrixXd gTeps_helper;
			Eigen::VectorXd gTeps, col_ones;
			col_ones.setOnes(n);
			gTeps_helper = D[k].dmp[j].theta_eps - col_ones*_dmps[j].getW().transpose();
			gTeps_helper = D[k].dmp[j].bases.cwiseProduct(gTeps_helper);
			gTeps = gTeps_helper.rowwise().sum();
			
// 			std::cout << gTeps.format(_HeavyFmt) << std::endl; exit(1);
			
			//compute g'g
			Eigen::VectorXd gTg;
			Eigen::MatrixXd gTg_helper;
			gTg_helper = D[k].dmp[j].bases.cwiseProduct(D[k].dmp[j].bases);
			gTg = gTg_helper.rowwise().sum();
			
// 			std::cout << gTg.format(_HeavyFmt) << std::endl; exit(1);
			
			//compute P*M*eps = P*g*g'*eps/(g'g) from prevous results
			Eigen::MatrixXd PMeps_member;
			matrix_small.setConstant(gTg.rows(), gTg.cols(), 1.e-10);
			PMeps_member = gTg + matrix_small;
			PMeps_member = gTeps.cwiseQuotient(PMeps_member);
			PMeps_member = P.col(k).cwiseProduct(PMeps_member);
			row_ones.setOnes(_n_basis);
			PMeps_member = PMeps_member*row_ones.transpose();
			PMeps[j][k] = D[k].dmp[j].bases.cwiseProduct(PMeps_member);
		}
	}
	
	//compute the parameter update per time step
	std::vector<Eigen::MatrixXd> dtheta;
	for(int i=0; i<_n_dmps; i++)
	{
		Eigen::MatrixXd dtheta_member;
		dtheta_member.setZero(n, _n_basis);
		
		for(int j=0; j<n_reps; j++)
			dtheta_member = dtheta_member + PMeps[i][j];
		
		dtheta.push_back(dtheta_member);
	}
	
	//average updates over time
	//the time weighting matrix (note that this done based on the true duration of the
	//movement, while the movement "recording" is done beyond D.duration). Empirically, this
	//weighting accelerates learning
	int m = (int)round(D[0].duration/D[0].dt);
	Eigen::VectorXd N;
    N.setZero(n);
	for(int i=0; i<m; i++)
		N(i)=m-i;
	Eigen::VectorXd col_ones;
	col_ones.setOnes(n-m);
	N.block(m,0,n-m,1) = col_ones;
    
	//the final weighting vector takes the kernel activation into account
	row_ones.setOnes(_n_basis);
	Eigen::MatrixXd W = (N*row_ones.transpose()).cwiseProduct(D[0].dmp[0].psi);
	
	//...and normalize through time
	col_ones.setOnes(n);
	W = W.cwiseQuotient(col_ones*W.colwise().sum());
    
	//compute the final parameter update for each DMP
	std::vector<Eigen::MatrixXd> processed_W;
	for(int i=0; i<_n_basis; i++)
	{
		Eigen::MatrixXd copyW;
		copyW = W;
		processed_W.push_back(copyW);
	}
	
	std::vector<Eigen::MatrixXd> dtheta_mult_W;
	for(int i=0; i<_n_dmps; i++)
	{
		Eigen::MatrixXd dtheta_mult_W_helper;
		dtheta_mult_W_helper = dtheta[i].cwiseProduct(processed_W[i]);
		dtheta_mult_W.push_back(dtheta_mult_W_helper.colwise().sum());
	}
    
	Eigen::MatrixXd final_dtheta;
	final_dtheta.setZero(_n_dmps,_n_basis);
	for(int i=0; i<_n_dmps; i++)
			final_dtheta.row(i) = dtheta_mult_W[i];
        
	//and update the parameters by changing w in _dmps
	for(int i=0; i<_n_dmps; i++)
	{
		_dmps[i].change_w(_dmps[i].getW() + final_dtheta.row(i).transpose());
// 		std::cout << "dtheta " << i << ": " << final_dtheta.row(i) << std::endl;
	}
	
	//update goal parameters in _dmps
	if(_update_goal)
	{
		std::cout << "updated goal:";
		Eigen::ArrayXd P0 = P.row(0);
		Eigen::VectorXd dg;
		dg.setZero(_n_dmps);
		for(int i=0; i<_n_dmps; i++)
		{
			for(int j=0; j<(int)D.size(); j++)
			{
				dg(i) += D[j].dmp[i].g_eps*P0(j);
// 				std::cout << D[j].dmp[i].g_eps << " ";
			}
			_protocol.goal[i] += dg(i);
			std::cout <<_protocol.goal[i] << " ";
// 			std::cout << std::endl;
		}
		std::cout << std::endl;
		std::cout << "delta_goal: " << dg.transpose() << std::endl;
	}
}

void PI2::eeStateCallback(const baxter_core_msgs::EndpointState& msg)
{
	if((int)_ee_current_time.size()>_ee_record_count)
	{
		if( std::abs((double)(msg.header.stamp.toSec() - _ee_current_time[_ee_record_count])) <0.01 ) // & ((msg.header.stamp - _ee_current_time).toSec() >=0) )
		{
// 			std::cout << "********************eeStateCallback******************************" << std::endl;
// 			std::cout << "ask for ee pose at time " << _ee_current_time[_ee_record_count] << std::endl;
// 			std::cout << "get ee pose at time " << msg.header.stamp.toSec() << std::endl;
// 			std::cout << "time difference is " << (double)(msg.header.stamp.toSec() - _ee_current_time[_ee_record_count]) << std::endl;
			
	// 		std::cout << "callback here: count = " << _ee_record_count << std::endl;
	// 		printf("position: %f, %f, %f\n", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	// 		printf("orientation: %f, %f, %f, %f\n", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
			
	// 		_average_ee_pose.position.x += msg.pose.position.x;
	// 		_average_ee_pose.position.y += msg.pose.position.y;
	// 		_average_ee_pose.position.z += msg.pose.position.z;
	// 		
	// 		_average_ee_pose.orientation.x += msg.pose.orientation.x;
	// 		_average_ee_pose.orientation.y += msg.pose.orientation.y;
	// 		_average_ee_pose.orientation.z += msg.pose.orientation.z;
	// 		_average_ee_pose.orientation.w += msg.pose.orientation.w;
			
			_average_ee_pose[_ee_record_count] = msg.pose;
			
			_average_ee_pose_time_stamp[_ee_record_count] = (double)msg.header.stamp.toSec();
			
			_ee_record_count++;
		}
	}
}




