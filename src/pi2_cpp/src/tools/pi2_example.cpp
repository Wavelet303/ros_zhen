/**
 * \file        pi2_example.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>
#include <ctime>

int main(int argc, char** argv)
{
	if(argc < 3)
	{
		std::cout << "[Usage]: pi2_example dmp_folder_name seg_id\n";
		exit(1);
	}
	
	ros::init(argc, argv, "pi2_example");
	ros::NodeHandle n;	
	
	PI2 pi2_example(7,100,true); // true to trigeer goal update
	pi2_example.setSegId(atoi(argv[2]));
	pi2_example.setDMPfolderName(argv[1]);
	pi2_example.loadDt(); // must load dt befor doing anything!
	
	pi2_example.readProtocol();
	
	clock_t begin = clock();
	// run learning process
	pi2_example.setROSNodeHandle(n);
	pi2_example.initializeW();
	pi2_example.setReferenceId();
	pi2_example.runProtocol();
	
	// run learned result
// 	pi2_example.loadLearnedW();
// 	pi2_example.loadLearnedGoal();
// 	pi2_example.runProtocolLearnedW();
	
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	
// 	std::cout << "elapsed time: " << elapsed_secs << std::endl;
	
	
	return 0;
}

