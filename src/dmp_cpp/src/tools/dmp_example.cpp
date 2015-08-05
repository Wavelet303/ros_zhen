/**
 * \file        dmp_example.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "dmp.h"
#include <iostream>

int main(int argc, char** argv)
{
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision);
	
	DMP dmp_example(10);
	dmp_example.reset_state(0);
	dmp_example.set_goal(1,1);
	
	Eigen::VectorXd cw;
	cw.setConstant(10,-1);
	
	std::vector< Eigen::VectorXd > test = dmp_example.run(1, 0.01, 0, 0, 1, 1, cw);
	
	
	Eigen::MatrixXd read_T= dmp_example.readMatrix("/home/zengzhen/Desktop/T.txt");
// 	std::cout << "read file T = " << read_T << std::endl; 
	
	Eigen::Map<Eigen::VectorXd> T(read_T.data(),read_T.cols()*read_T.rows(),1);
// 	std::cout << "target T is " << T << std::endl;
	
	dmp_example.batch_fit(3.0, 0.01, T);
	
	return 0;
}

