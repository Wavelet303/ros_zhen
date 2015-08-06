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

PI2::PI2(int n_dmps, int n_basis)
{
	_n_dmps = n_dmps;
	_n_basis = n_basis;
	
	for(int i=0; i<_n_dmps; i++)
	{
		DMP dmp(_n_basis);
		_dmps.push_back(dmp);
	}
	
	_HeavyFmt = Eigen::IOFormat(Eigen::FullPrecision);
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

void PI2::runProtocol()
{
	double dt = 0.01;
	int n = 2*(int)round(_protocol.duration / dt);
	
	PI2Data D(n, _n_dmps, _n_basis, _protocol.duration, dt, _protocol.goal);
	
	PI2Data D_eval = D;
	PI2Protocol p_eval = _protocol;
	p_eval.reps = 1;
	p_eval.stdv = 0;
	p_eval.n_reuse = 0;
	
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
	
	for(int i=0; i<_protocol.updates;i++)
	{
		run_rollouts(D_eval_series, p_eval,1, &infile);
		
		//compute all costs in batch form, as this is faster in matlab
		Eigen::MatrixXd R_eval = cost(D_eval_series);
		
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
	
	std::cout << "cost trace = " << T.col(1).transpose().format(_HeavyFmt) << std::endl;
}

void PI2::run_rollouts(std::vector<PI2Data>& D, PI2Protocol p, double noise_mult, std::ifstream* infile)
{
	std::string line;
	
	double dt = D[0].dt;
	
	//run roll-outs
	int start = p.n_reuse;
	if(D[0].dmp[0].psi(0,0)==0) //indicates very first batch of run_rollouts
		start = 0;
	
	// generate desired trajectory
	for(int k = start; k<p.reps; k++ )
	{
		//reset the DMP
		for(int j=0; j<_n_dmps; j++)
		{
			_dmps[j].reset_state(p.start[j]);
			_dmps[j].set_goal(p.goal[j], 1);
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
// 							printf("n=%d: %.63f\n", n, epsilon(0));
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
// 								printf("n=%d: %.63f\n", n, epsilon(ind_basis));
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
			}
		}
	}
	
	//command generated trajectory to Baxter
	for(int k = start; k<p.reps; k++ )
	{
		
	}
}

Eigen::MatrixXd PI2::cost(std::vector< PI2Data > D)
{	
	//implement the "acc2_exp.m" cost function + terminal cost
	int n_reps = D.size();
	int n = D[0].dmp[0].y.size(); //the length of a trajectory in time steps
	int n_real = (int)round(D[0].duration/D[0].dt); // the duration of the core trajectory in time steps -- everything beyond this time belongs to the terminal cost
	
	Eigen::MatrixXd R;
	R.setZero(n, n_reps);
	
	//compute cost
	for(int k=0; k<n_reps; k++)
	{
		Eigen::VectorXd r, rt;
		r.setZero(n_real);
		rt.setZero(n-n_real);
		
		for(int i=0; i<_n_dmps; i++)
		{
			//cost during trajectory
			Eigen::VectorXd ydd;
			ydd = D[k].dmp[i].ydd.block(0,0,n_real,1);
			ydd = ydd.block(0,0,n_real,1);
			ydd = ydd.cwiseAbs();
			ydd = ydd*(-0.01);
			ydd = ydd.array().exp();
			Eigen::VectorXd ones_col;
			ones_col.setOnes(ydd.size(),1);
			r = r + (ones_col-ydd)/n_real;
			
			//terminal cost: to be decided
		}
		
		Eigen::VectorXd rrt;
		rrt.setZero(r.size()+rt.size());
		rrt.block(0,0,r.size(),1) = r;
		rrt.block(r.size(),0,rt.size(),1) = rt;
		R.col(k) = rrt;
	}
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
}








