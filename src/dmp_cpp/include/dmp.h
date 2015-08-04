/**
 * \file        dmp.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       dynamic movement primitives class
 */

#ifndef DMP_H
#define DMP_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>

#define MAXBUFSIZE  ((int) 1e6)

class DMP {
public:
	/** \brief Constructor.
	*/
	DMP(int n_basis);
	
	/** \brief reset state.
	*   \param[in] y (optional)starting state (default y = 0) 
	*/
	void reset_state(double y=0);
	
	/** \brief reset state.
	 *  \param[in] goal reset goal value
	 *  \param[in] flag reset x to 1(flag=1) or not(flag=0) 
	 */
	void set_goal(double goal, bool flag);
	
	/** \brief Set the scale factor of the movement
	 *   \param[in] scale
	 */
	void set_scale(double scale);
	
	/** \brief run the dmp
	 *  \param[in] duration movement time
	 *  \param[in] dt integration time step (tau is roughly 1/movement time until convergence the goal, here tau=1/tau in the DMP document)
	 *  \param[in] ct (optional)coupling term for transformation system (default=0)
	 *  \param[in] cc (optional)coupling term for canonical system (default=0)
	 *  \param[in] ct_tau (optional)coupling term for transformation system's time constant (default=1)
	 *  \param[in] cc_tau (optional)coupling term for canonical system's time constant (default=1)
	 *  \param[in] cw (optional)additive coupling term for parameters (default=0)
	 *  \return    vector(y; yd; ydd; values of multipliers of w in forcing term)
	 */
	std::vector<Eigen::VectorXd> run(double duration, double dt, double ct=0, double cc=0, double ct_tau=1, double cc_tau=1, Eigen::VectorXd cw=Eigen::Matrix<double,-1,1>());
	
	/** \brief change the weights of basis functions in the dmp
	 *  \param[in] w
	 */
	void change_w(Eigen::VectorXd w);
	
	/** \brief fit the dmp to a complete trajectory in batch mode
	 *  \param[in] duration movement time (tau is roughly 1/movement time until convergence the goal, here tau=1/tau in the DMP document)
	 *  \param[in] dt sample time step in given trajectory
	 *  \param[in] T target trajectory for y
	 *  \param[in] Td target trajectory for yd (optional, will be generated as dT/dt otherwise)
	 *  \param[in] Tdd target trajectory for ydd (optional, will be generated as dTd/dt otherwise)
	 */
	std::vector<Eigen::VectorXd> batch_fit(double duration, double dt, Eigen::VectorXd T, Eigen::VectorXd Td=Eigen::Matrix<double,-1,1>(), Eigen::VectorXd Tdd=Eigen::Matrix<double,-1,1>());
	
	Eigen::MatrixXd readMatrix(const char *filename);
	
	inline Eigen::VectorXd getW() {return _w;};
	
	inline Eigen::VectorXd getPsi() {return _psi;};
	
private:
	double _alpha_z;
	double _beta_z;
	double _alpha_x;
	double _alpha_g;
	
	int _n_basis;
	Eigen::VectorXd _basis_centers; //center of basis kernel functions (evenly spaced in time) 
	Eigen::VectorXd _basis_sigmas; //variance of basis kernel functions (std_var = 0.2*|c_i-c_{i-1}|)
	Eigen::VectorXd _w; //weights of basis functions (policy parameters in RL)
	Eigen::VectorXd _psi; //the value of basis functions
	
	double _g;
	double _gd;
	double _G;

	double _y0; //the current start state of the primitive
	double _A; //the orginal amplitude (max(y)-min(y)) when the primitive was fit
	double _dG; //the original goal amplitude (G-y0) when the primitive was fit
	double _s; //the scale factor for the nonlinear function
	
	double _z;
	double _y;
	double _x;
	double _zd;
	double _yd;
	double _ydd;
	double _xd;
};

#endif  // DMP_H
