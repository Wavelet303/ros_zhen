/**
 * \file        pi2_example.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "pi2.h"
#include <iostream>

int main(int argc, char** argv)
{
	PI2 pi2_example(2,10);
	
	pi2_example.readProtocol(std::string(argv[1]));
	pi2_example.runProtocol();
	
	return 0;
}

