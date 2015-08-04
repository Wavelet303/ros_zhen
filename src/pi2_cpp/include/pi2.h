/**
 * \file        pi2.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       PI^2 learning class
 */

#ifndef PI2_H
#define PI2_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include "dmp.h"

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
	
	PI2DMP(int steps, int n_basis)
	{
		y.setZero(steps);
		yd.setZero(steps);
		ydd.setZero(steps);
		bases.setZero(steps, n_basis);
		theta_eps.setZero(steps, n_basis);
		psi.setZero(1,n_basis);
	}
};



struct PI2Data{
	std::vector<PI2DMP> dmp;
	double duration;
	double dt;
	std::vector<double> goal;
	
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
	}
};

class PI2 {
public:
	/** \brief Constructor.
	*/
	PI2(int n_dmps, int n_basis);
	
	/** \brief read protocol file.
	*   \param[in] filename 
	*/
	void readProtocol(std::string protocol_name);
	
	/** \brief run loaded protocol
	 */
	void runProtocol();
	
	/** \brief a dedicated function to tun multiple roll-outs using the specifications in D and p
	 *         noise_mult allows decreasing the noise with the number of roll-outs, which gives smoother converged
	 *         performance (but it is not needed for convergence)
	 *  \param[in] D data structures to store each roll-outs data
	 *  \param[in] p protocol to follow
	 *  \param[in] noise_mult noise multiplier applied to exploration noise
	 */
	void run_rollouts(std::vector<PI2Data>& D, PI2Protocol p, double noise_mult);
	
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
	
	// random number generator based on normal distribution
	std::default_random_engine _generator;
	std::normal_distribution<double> _distribution;
};

#endif  // PI2_H
