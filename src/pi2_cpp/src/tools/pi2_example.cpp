/**
 * \file        pi2_example.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		std::cout << "Please input protocol file\n";
		exit(1);
	}
	
	ros::init(argc, argv, "pi2_example");
	ros::NodeHandle n;
	
	PI2 pi2_example(2,10);
	
	pi2_example.readProtocol(std::string(argv[1]));
	
	clock_t begin = clock();
	pi2_example.setROSNodeHandle(n);
	pi2_example.runProtocol();
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	
// 	std::cout << "elapsed time: " << elapsed_secs << std::endl;
	
	
	return 0;
}

