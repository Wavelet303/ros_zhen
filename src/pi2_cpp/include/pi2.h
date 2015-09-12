/**
 * \file        pi2.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       PI^2 learning class
 */

#ifndef PI2_H
#define PI2_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include "dmp.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/exceptions.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/EndpointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#define MAXBUFSIZE  ((int) 1e6)

struct PI2Protocol {
	std::vector<double> start;
	std::vector<double> goal;
	double duration, stdv;
	int reps, updates,basis_noise, n_reuse;
	char cost_function[50];
};

struct PI2DMP{
	Eigen::VectorXd y;
	Eigen::VectorXd yd;
	Eigen::VectorXd ydd;
	Eigen::MatrixXd bases;
	Eigen::MatrixXd theta_eps;
	Eigen::MatrixXd psi;
	double g_eps;
	
	PI2DMP(int steps, int n_basis)
	{
		y.setZero(steps);
		yd.setZero(steps);
		ydd.setZero(steps);
		bases.setZero(steps, n_basis);
		theta_eps.setZero(steps, n_basis);
		psi.setZero(steps,n_basis);
		g_eps = 0.0;
	}
};



struct PI2Data{
	std::vector<PI2DMP> dmp;
	double duration;
	double dt;
	std::vector<double> goal;
	double user_input_cost;
	
	Eigen::MatrixXd q;
	Eigen::MatrixXd qd;
	Eigen::MatrixXd qdd;
	
	PI2Data(int steps, int n_dmps, int n_basis, double input_duration, double input_dt, std::vector< double > input_goal)
	{
		for(int i=0; i<n_dmps; i++)
		{
			PI2DMP dmp_temp(steps, n_basis);
			dmp.push_back(dmp_temp);
		}
		duration = input_duration;
		dt = input_dt;
		goal = input_goal;
		
		q.setZero(steps, n_dmps);
		qd.setZero(steps, n_dmps);
		qdd.setZero(steps, n_dmps);
	}
};

class PI2 {
public:
	/** \brief Constructor.
	*/
	PI2(int n_dmps, int n_basis, bool update_goal=false);
	
	/** \brief read protocol file.
	*   \param[in] filename 
	*/
	void readProtocol(std::string protocol_name);
	
	/** \brief initialize policy parameters (w for DMPs for each DOF) by fitting to demonstration
	 *  \\param[in] dmp_folder_name folder that stores the trajectory
	 */
	void initializeW(char* dmp_folder_name);
	
	/** \brief load learned policy parameters (w for DMPs for each DOF)
	 *  \param[in] dmp_folder_name folder that stores the learned w
	 */
	void loadLearnedW(char* dmp_folder_name);
	
	/** \brief set _reference_id (block id that was selected as reference frame in MATLAB)
	 *  \\param[in] dmp_folder_name folder that stores the trajectory
	 */
	void setReferenceId(char* dmp_folder_name);
	
	/** \brief write learned goal to file
	 *  \param[in] dmp_folder_name folder to store the learned goal
	 */
	void writeGoalToFile(char* dmp_folder_name);
	
	/** \brief load learned goal
	 *  \param[in] dmp_folder_name folder that stores the learned goal
	 */
	void loadLearnedGoal(char* dmp_folder_name);
	
	/** \brief run loaded protocol
	 */
	void runProtocol();
	
	/** \brief run loaded protocol with learned w
	 */
	void runProtocolLearnedW();
	
	/** \brief set joint position command setPublisher
	 *         set endpoint position subscriber
	 *  \param[in] publisher initialized joint command publisher from ros node
	 *  \param[in] subscribe initialized endpoint position subscriber from ros node
	 *  \param[in] client initialized IK service client
	 *  \param[in] srv initialized IK service server 
	 */
// 	void setROS(ros::Publisher publisher, ros::Subscriber subscriber, ros::ServiceClient client, baxter_core_msgs::SolvePositionIK srv);
	
	/** \brief set joint position command setPublisher
	 *         set endpoint position subscriber
	 *  \param[in] n initialized ros nodehandle 
	 */
	void setROSNodeHandle(ros::NodeHandle& n);
	
	/** \brief a dedicated function to tun multiple roll-outs using the specifications in D and p
	 *         noise_mult allows decreasing the noise with the number of roll-outs, which gives smoother converged
	 *         performance (but it is not needed for convergence)
	 *  \param[in] D data structures to store each roll-outs data
	 *  \param[in] p protocol to follow
	 *  \param[in] noise_mult noise multiplier applied to exploration noise
	 */
	void run_rollouts(std::vector<PI2Data>& D, PI2Protocol p, double noise_mult, std::ifstream* infile = NULL);
	
	/** \brief run desired trajectory stored in D on Baxter and store executed trajectory in D
	 *  \param[in] D data structures to store each roll-outs data (desired & executed)
	 *  \param[in] trial_index ith trial
	 */
	void run_baxter(std::vector<PI2Data>& D, int trial_index);
	
	/** \brief calculate cost of given data of each roll-outs
	 *  \param[in] D data structures to store each roll-outs data
	 *  \return matrix with rt at each step for each roll-outs(column)
	 */
	Eigen::MatrixXd cost(std::vector<PI2Data> D);
	
	/** \brief update w based on roll-outs data and corresponding cost
	 *  \param[in] D data structures to store each roll-outs data
	 *  \param[in] R matrix of cost for for each roll-outs(col) at each time step(row)
	 */
	void updatePI2(std::vector<PI2Data> D, Eigen::MatrixXd R);
	
private:
	int _n_basis;
	int _n_dmps;
	
	std::vector<DMP> _dmps;
	
	PI2Protocol _protocol;
	double _dt; //unit time
	
	// random number generator based on normal distribution
	std::default_random_engine _generator;
	std::normal_distribution<double> _distribution;
	
	Eigen::IOFormat _HeavyFmt;
	
	bool _ROS_initialized;
	
	ros::NodeHandle _n;
	ros::Publisher _publisher;
	ros::Publisher _gripper_publiser;
	ros::Subscriber _subscriber;
	ros::ServiceClient _client;
	baxter_core_msgs::SolvePositionIK _srv;
	std::vector<geometry_msgs::Pose> _average_ee_pose;
	std::vector<double> _average_ee_pose_time_stamp;
	int _ee_record_count;
	std::vector<double> _ee_current_time;
	
	/** \brief callback function of subscriber on end point pose
	 *  \param[in] msg received end point pose msg
	 */
	void eeStateCallback(const baxter_core_msgs::EndpointState& msg);
	
	bool _update_goal;
	char* _dmp_folder_name;
	
	// reference frame from state abstraction selection (MATLAB)
	int _reference_id;
	tf::StampedTransform _reference_to_base_transform;
};

#endif  // PI2_H
