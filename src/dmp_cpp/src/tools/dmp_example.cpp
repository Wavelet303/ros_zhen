/**
 * \file        dmp_example.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "dmp.h"
#include <iostream>

#include "discpp.h"

int main(int argc, char** argv)
{
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision);
	
	DMP dmp_example(10);
	dmp_example.reset_state(0);
	dmp_example.set_goal(1,1);
	
	Eigen::VectorXd cw;
	cw.setConstant(10,0.1);
	
	std::vector< Eigen::VectorXd > test = dmp_example.run(1, 0.01, 0, 0, 1, 1, cw);
	
// 	std::cout << "y = " << test[0].format(HeavyFmt) << std::endl;
// 	std::cout << "yd = " << test[1].format(HeavyFmt) << std::endl;
// 	std::cout << "ydd = " << test[2].format(HeavyFmt) << std::endl;
// 	std::cout << "w_multipliers" << test[3].format(HeavyFmt) << std::endl;
	
	
	Eigen::MatrixXd read_T= dmp_example.readMatrix("/home/zengzhen/Desktop/T.txt");
// 	std::cout << "read file T = " << read_T << std::endl; 
	
	Eigen::Map<Eigen::VectorXd> T(read_T.data(),read_T.cols()*read_T.rows(),1);
// 	std::cout << "target T is " << T << std::endl;
	
	dmp_example.batch_fit(3.0, 0.01, T);
	
	// 
	int n = 100, i, ic;
	double fpi = 3.1415926 / 180.0, step, x;
	double xray[100], y1ray[100], y2ray[100];
	Dislin g;
	
	step = 360. / (n - 1);
	
	for (i = 0; i < n; i++)
	{ xray[i] = i * step;
		x = xray[i] * fpi;
		y1ray[i] = sin (x);
		y2ray[i] = cos (x);
	}
	
	g.metafl ("cons");
	g.scrmod ("revers");
	g.disini ();
	g.pagera ();
	g.complx ();
	g.axspos (450, 1800);
	g.axslen (2200, 1200);
	
	g.name   ("X-axis", "x");
	g.name   ("Y-axis", "y");
	
	g.labdig (-1, "x");
	g.ticks  (9, "x");
	g.ticks  (10, "y");
	
	g.titlin ("Demonstration of CURVE", 1);
	g.titlin ("SIN(X), COS(X)", 3);
	
	ic=g.intrgb (0.95,0.95,0.95);
	g.axsbgd (ic);
	
	g.graf   (0.0, 360.0, 0.0, 90.0, -1.0, 1.0, -1.0, 0.5);
	g.setrgb (0.7, 0.7, 0.7);
	g.grid   (1, 1);
	
	g.color  ("fore");
	g.height (50);
	g.title  ();
	
	g.color  ("red");
	g.curve  (xray, y1ray, n);
	g.color  ("green");
	g.curve  (xray, y2ray, n);
	g.disfin ();
	
	return 0;
}

