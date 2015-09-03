/**
 * \file        read_matrix.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 * \brief test whether dmp::readMatrix works fine with automatically recorded file
 */

#include "dmp.h"
#include <iostream>

int main(int argc, char** argv)
{
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision);
	
	DMP dmp_example(10);
	Eigen::MatrixXd read_T= dmp_example.readMatrix(argv[1]);
	std::cout << "size of T = " << read_T.cols() << " " << read_T.rows() << std::endl;
	std::cout << "read file T = " << read_T << std::endl; 
	
// 	Eigen::Map<Eigen::VectorXd> T(read_T.data(),read_T.cols()*read_T.rows(),1);
// 	std::cout << "target T is " << T << std::endl;
	
	return 0;
}

