/**
 * \file        dmp.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "dmp.h"
#include <iostream>
#include <math.h>
#include "discpp.h"

DMP::DMP(int n_basis)
{
	//set parameters for transformation and canonical system
	_alpha_z	= 25;
	_beta_z		= _alpha_z/4.0;
	_alpha_x	= _alpha_z/3.0;
	_alpha_g    = _alpha_z/2.0;
	
	//set center of basis kernel functions (evenly spaced in time) 
	_n_basis = n_basis;
	_basis_centers.setLinSpaced(_n_basis, 0.0, 1.0);
	_basis_centers = -_alpha_x * _basis_centers;
	Eigen::Map<Eigen::ArrayXd> centers_array(_basis_centers.data(),_n_basis,1);
	centers_array = centers_array.exp();
	
	//set variance of basis kernel functions (std_var = 0.2*|c_i-c_{i-1}|)
	_basis_sigmas.setZero(_n_basis);
	_basis_sigmas.block(0,0,_n_basis-1,1) = _basis_centers.block(1,0,_n_basis,1);
	_basis_sigmas = _basis_sigmas - _basis_centers;
	_basis_sigmas[_n_basis-1] = _basis_sigmas[_n_basis-2];
	_basis_sigmas = _basis_sigmas*0.2;
	Eigen::Map<Eigen::ArrayXd> sigmas_array(_basis_sigmas.data(),_n_basis,1);
	sigmas_array = sigmas_array.pow(2);
	sigmas_array = 1/sigmas_array;
	
	//initialize weights of basis functions (policy parameters in RL)
	_w.setZero(_n_basis);
	
	//initialize the value of basis functions
	_psi.setZero(_n_basis);
	
	//initialize the current goal state
	_g = 0;
	_gd = 0;
	_G = 0;
	
	//initialize the current start state of the primitive
	_y0 = 0;
	
	//initialize the orginal amplitude (max(y)-min(y)) when the primitive was fit
	_A = 0;
	
	//initialize the original goal amplitude (G-y0) when the primitive was fit
	_dG = 0; 
	
	//initialize the scale factor for the nonlinear function
	_s = 1; 
	
	//initialize the state variables
	_z = 0;
	_y = 0;
	_x = 0;
	_zd = 0;
	_yd = 0;
	_ydd = 0;
	_xd = 0;
}

void DMP::reset_state(double y)
{
	//initialize the state variables
	_z = 0;
	_y = y;
	_x = 0;
	_zd = 0;
	_yd = 0;
	_ydd = 0;
	_xd = 0;
	
	//initialize the goal state
	_G = y;
	_g = y;
	_gd = 0;
	_y0 = y;
	_s = 1;
}

void DMP::set_goal(double goal, bool flag)
{
	_G = goal;
	_g = _G;
	
	if(flag)
	{
		_x = 1;
		_y0 = _y;
	}
	if(_A!=0) //check whetehr dmp has been fit
	{
		if(_A/std::abs(_dG)+1e-10 <=2.0)
		{
			_s = (_G-_y0)/_dG;
		}
	}
}

void DMP::set_scale(double scale)
{
	_s = scale;
}

std::vector< Eigen::VectorXd > DMP::run(double duration, double dt, double ct, double cc, double ct_tau, double cc_tau, Eigen::VectorXd cw)
{
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision);
	
	double tau = 1.0/duration;
	
	//cw set to 0 if not given
	if(cw.size()==0)
		cw.setZero(_n_basis);
	
	//basis function values
	Eigen::VectorXd copyX;
	copyX.setConstant(_n_basis, _x);
	copyX = copyX - _basis_centers;
	Eigen::Map<Eigen::ArrayXd> psi_array(copyX.data(),_n_basis,1);
	psi_array = psi_array.pow(2);
	Eigen::Map<Eigen::ArrayXd> sigmas_array(_basis_sigmas.data(),_n_basis,1);
	psi_array = psi_array*sigmas_array;
	psi_array = -0.5*psi_array;
	psi_array = psi_array.exp();
	_psi = copyX;
	
	//amplitude
	double amp = _s;
	
	//forcing term
	double in = _x;
	Eigen::VectorXd noiseW = _w + cw;
	Eigen::Map<Eigen::ArrayXd> nominator_array(noiseW.data(),_n_basis,1);
	nominator_array = nominator_array*psi_array;
	nominator_array = _x*nominator_array;
	Eigen::ArrayXd safe_psi_array;
	safe_psi_array.setConstant(_n_basis, 1e-10);
	safe_psi_array = safe_psi_array + psi_array;
	
	double f = nominator_array.sum()/safe_psi_array.sum()*amp;
	
	//canonical system
	_xd = (_alpha_x*(0-_x)+cc)*tau*cc_tau;
	
	//transformation system
	_zd = (_alpha_z*(_beta_z*(_g-_y)-_z)+f+ct)*tau*ct_tau;
	_yd = _z*tau*ct_tau;
	_ydd = _zd*tau*ct_tau;
	
	_gd = _alpha_g*(_G-_g);
	
	//canonical system: update
	_x = _xd*dt + _x;
	
	//transformation system: update
	_z = _zd*dt + _z;
	_y = _yd*dt + _y;
	
	_g = _gd*dt + _g;
	
	//output y; yd; ydd; values of multipliers of w in forcing term
	std::vector<Eigen::VectorXd> output;
	
	Eigen::VectorXd y;
	y.setConstant(1, _y);
	output.push_back(y);
	
	Eigen::VectorXd yd;
	yd.setConstant(1, _yd);
	output.push_back(yd);
	
	Eigen::VectorXd ydd;
	ydd.setConstant(1, _ydd);
	output.push_back(ydd);
	
	Eigen::VectorXd w_multipliers;
	w_multipliers = _psi*in/safe_psi_array.sum()*amp;
	output.push_back(w_multipliers);
	
	return output;
}

void DMP::change_w(Eigen::VectorXd w)
{
	_w = w;
}

std::vector< Eigen::VectorXd > DMP::batch_fit(double duration, double dt, Eigen::VectorXd T, Eigen::VectorXd Td, Eigen::VectorXd Tdd)
{
	double tau = 1.0/duration;
	int T_length = T.size();
	
	std::cout << "T_length = " << T_length << std::endl;
	
	if(Td.size()==0)
	{
		Td.setZero(T_length);
		Td.block(0,0,T_length-1,1) = T.block(1,0,T_length,1);
		Td = Td - T;
		Td[T_length-1] = 0;
		Td = Td/dt;
	}	
	
	if(Tdd.size()==0)
	{
		Tdd.setZero(T_length);
		Tdd.block(0,0,T_length-1,1) = Td.block(1,0,T_length,1);
		Tdd = Tdd - Td;
		Tdd[T_length-1] = 0;
		Tdd = Tdd/dt;
	}
	
	double y0 = T(0);
	double g = y0;
	double goal = T(T_length-1);
	g = goal;
	double A = T.maxCoeff() - T.minCoeff();
	
	Eigen::VectorXd X, G;
	X.setZero(T_length);
	G.setZero(T_length);
	
	double x = 1;
	
	for(int i=0; i<T_length; i++)
	{
		X(i) = x;
		G(i) = g;
		
		double xd = _alpha_x*(0-x)*tau;
		double gd = (goal - g)*_alpha_g;
		
		x = xd*dt + x;
		g = gd*dt + g;
	}
	
	//the regression target
	_dG = goal - y0;
	_A = T.maxCoeff() - T.minCoeff();
	_s = 1; // for fitting an ew primitive, the scale factor is always equal to 1
	
	double amp = _s;
	Eigen::Map<Eigen::ArrayXd> T_array(T.data(),T_length,1);
	Eigen::Map<Eigen::ArrayXd> Td_array(Td.data(),T_length,1);
	Eigen::Map<Eigen::ArrayXd> Tdd_array(Tdd.data(),T_length,1);
	Eigen::Map<Eigen::ArrayXd> G_array(G.data(),T_length,1);
	Eigen::VectorXd copyX = X;
	Eigen::Map<Eigen::ArrayXd> X_array(copyX.data(),T_length,1);
	
	//compute the target forcing term
	Eigen::ArrayXd Ft_array;
	Ft_array = (Tdd_array/std::pow(tau,2) - _alpha_z*(_beta_z*(G_array-T_array)-Td_array/tau)) / amp;
	Eigen::Map<Eigen::VectorXd> Ft(Ft_array.data(),T_length,1);
	
	//compute the weights for each local model along the trajectory
	Eigen::VectorXd row_ones;
	Eigen::VectorXd col_ones;
	row_ones.setOnes(_n_basis);
	col_ones.setOnes(T_length);
	Eigen::MatrixXd x_minus_centers = X*row_ones.transpose() - col_ones*_basis_centers.transpose();
	Eigen::MatrixXd copyD = col_ones*_basis_sigmas.transpose();
	
	Eigen::Map<Eigen::ArrayXXd> x_minus_centers_array(x_minus_centers.data(), x_minus_centers.rows(), x_minus_centers.cols());
	Eigen::Map<Eigen::ArrayXXd> copyD_array(copyD.data(), copyD.rows(), copyD.cols());
	x_minus_centers_array = x_minus_centers_array.pow(2);
	Eigen::ArrayXXd PSI_array = -0.5*x_minus_centers_array*copyD_array;
	PSI_array = PSI_array.exp();
	Eigen::Map<Eigen::MatrixXd> PSI(PSI_array.data(), PSI_array.rows(), PSI_array.cols());
	
	//compute the regression
	copyX = X;
	X_array = X_array.pow(2);
	Eigen::MatrixXd copyXsquare = copyX*row_ones.transpose();
	Eigen::MatrixXd sx2 = copyXsquare.cwiseProduct(PSI);
	Eigen::ArrayXd sx2_array = sx2.colwise().sum();
	
	copyX = X;
	X_array = X_array*Ft_array;
	copyXsquare = copyX*row_ones.transpose();
	Eigen::MatrixXd sxtd = copyXsquare.cwiseProduct(PSI);
	Eigen::ArrayXd sxtd_array = sxtd.colwise().sum();
	
	Eigen::ArrayXd w_array = sxtd_array/(sx2_array+1e-10);
	Eigen::Map<Eigen::VectorXd> w(w_array.data(), _n_basis, 1);
	_w = w;
	
	//compute the prediction
	Eigen::MatrixXd xmulw = X*_w.transpose();
	xmulw = xmulw.cwiseProduct(PSI);
	Eigen::ArrayXd nominator = xmulw.rowwise().sum();
	Eigen::ArrayXd denominator = PSI.rowwise().sum();
	Eigen::ArrayXd F = nominator/denominator*amp;
	
	double z=0;
	double zd = 0;
	double y = y0;
	double yd = 0;
	Eigen::VectorXd Y, Yd, Ydd;
	Y.setZero(T_length);
	Yd.setZero(T_length);
	Ydd.setZero(T_length);
	
	zd = (_alpha_z*(_beta_z*(G(0)-y)-z)+F(0))*tau;
	yd = 0;
	
	Ydd(0) = zd*tau;
	Yd(0) = yd;
	Y(0) = y;
	
	for(int i=1; i<T_length; i++)
	{
		y = yd*dt + y;
		z = zd*dt + z;
		
		Yd(i) = z*tau;
		Y(i)  = y;
		
		zd = (_alpha_z*(_beta_z*(G(i)-y)-z)+F(i))*tau;
		yd = z*tau;
		
		Ydd(i) = zd*tau;
	}
	
	//output y; yd; ydd; values of multipliers of w in forcing term
	std::vector<Eigen::VectorXd> output;
	output.push_back(Y);
	output.push_back(Yd);
	output.push_back(Ydd);
	
	//calculate regression rms error
	Eigen::VectorXd diffTY = T-Y;
	diffTY = diffTY.cwiseProduct(diffTY);
	Eigen::VectorXd error_vector = diffTY.colwise().sum();
	
	double error = error_vector(0);
	error = std::sqrt(error/T_length);
	std::cout << "DMP regression RMS error = " << error << std::endl;
	
	Dislin dislin_plot;
	double time[T_length], T_plot[T_length], Y_plot[T_length];
	for(int i=0; i<T_length; i++)
	{
		T_plot[i] = T(i);
		Y_plot[i] = Y(i);
		time[i] = dt*i;
	}
	dislin_plot.titlin ("DMP batch fitting", 1);
	dislin_plot.color  ("fore");
	dislin_plot.height (50);
	dislin_plot.title  ();
	
	dislin_plot.name   ("time/s", "x");
	dislin_plot.name   ("position/m", "y");
	
	dislin_plot.color  ("green");
	dislin_plot.curve  (time, T_plot, T_length);
	dislin_plot.color  ("red");
	dislin_plot.curve  (time, Y_plot, T_length);
// 	dislin_plot.disfin ();
	
	return output;
}

Eigen::MatrixXd DMP::readMatrix(const char *filename)
{
	int cols = 0, rows = 0;
	double buff[MAXBUFSIZE];
	
	// Read numbers from file into buffer.
	std::ifstream infile;
	infile.open(filename);
	while (! infile.eof())
	{
		std::string line;
		getline(infile, line);
		
		int temp_cols = 0;
		std::stringstream stream(line);
		while(! stream.eof())
			stream >> buff[cols*rows+temp_cols++];
		
		if (temp_cols == 0)
			continue;
		
		if (cols == 0)
			cols = temp_cols;
		
		rows++;
	}
	
	infile.close();
	
	rows--;
	
	// Populate matrix with numbers.
	Eigen::MatrixXd result(rows,cols);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			result(i,j) = buff[ cols*i+j ];
		
	return result;
}




