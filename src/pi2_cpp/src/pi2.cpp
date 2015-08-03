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

PI2::PI2(int n_dmps, int n_basis)
{
	_n_dmps = n_dmps;
	_n_basis = n_basis;
	
	for(int i=0; i<_n_dmps; i++)
	{
		DMP dmp(_n_basis);
		_dmps.push_back(dmp);
	}
	
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
		}
	}
// 	std::cout << _protocol.cost_function << "\n";
}

void PI2::runProtocol()
{
	double dt = 0.01;
	int n = (int)_protocol.duration / dt;
	
	PI2DMP dmp_temp;
	dmp_temp.y.setZero(n,1);
	dmp_temp.yd.setZero(n,1);
	dmp_temp.ydd.setZero(n,1);
	dmp_temp.bases.setZero(n, _n_basis);
	dmp_temp.theta_eps.setZero(n, _n_basis);
	dmp_temp.psi.setZero(1,_n_basis);
	
	PI2Data D;
	for(int i=0; i<_n_dmps; i++)
		D.dmp.push_back(dmp_temp);
	D.duration = _protocol.duration;
	D.dt = dt;
	D.goal = _protocol.goal;
	
	PI2Data D_eval = D;
	PI2Protocol p_eval = _protocol;
	p_eval.reps = 1;
	p_eval.stdv = 0;
	p_eval.n_reuse = 0;
	
	//one data structure for each repetition
	std::vector<PI2Data> D_series;
	for(int i=0; i<_protocol.reps; i++)
	{
		PI2Data copyD = D;
		D_series.push_back(copyD);
	}
	
	//used to store the cost as learning happens
	Eigen::MatrixXd T;
	T.setZero(_protocol.updates+1, 2);
	
	for(int i=0; i<_protocol.updates;i++)
	{
		double noise_mult=0.99;
		run_rollouts(D_series, p_eval,noise_mult);
	}
	
}

PI2Data PI2::run_rollouts(std::vector<PI2Data> D, PI2Protocol p, double noise_mult)
{
	double dt = D[0].dt;
	
	//run roll-outs
	int start = p.n_reuse;
	if(D[0].dmp[0].psi(0,0)==0) //indicates very first batch of run_rollouts
		start = 0;
	
	for(int k = start; k<p.reps; k++ )
	{
		//reset the DMP
		for(int j=0; j<_n_dmps; j++)
		{
			_dmps[j].reset_state(p.start[j]);
			_dmps[j].set_goal(p.goal[j], 1);
		}
		
		
		//integrate through the duration
		for(int n=0; n<(int)p.duration/dt; n++)
		{
			double std_eps = p.stdv * noise_mult;
			
			for(int j=0; j<_n_dmps; j++)
			{
				Eigen::VectorXd epsilon;
				epsilon.setZero(_n_basis,1);
				
				//generate noise_mult
				if(!p.basis_noise) //this case adds noise at every time step
				{
					for(int test=0; test<10; test++)
						epsilon(test) = _distribution(_generator)*std_eps;
					
				}else{ //this case only adds noise for the most active basis function,
					   //and noise does not change during hte activity of the basis function
					if(n==0)
					{
						epsilon(0)=_distribution(_generator)*std_eps;
						epsilon.block(1,0, _n_basis-1,1) = epsilon.block(1,0, _n_basis-1,1)*0;
					}else{
						//what is the max activated basis function from the previous time step?
						Eigen::ArrayXd psi_row_array = D[k].dmp[j].psi.row(n-2);
						psi_row_array.setOnes(10,1);
						psi_row_array(0)=11;
						
						std::vector<double> psi_row(psi_row_array.data(), psi_row_array.data()+psi_row_array.rows()*psi_row_array.cols());
						int psi_N = sizeof(psi_row)/sizeof(double);
						int myints[] = {3,7,2,5,6,4,9};
						int max_index;
						std::distance(psi_row, std::max_element(std::begin(psi_row), std::end(psi_row)));
						std::cout << "max_indx = " << max_index << std::endl;
					}
// 					std::cout << epsilon << std::endl;
					
				}
				
			}
		}
	}
	
	PI2Data testD;
	return testD;
}










