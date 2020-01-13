//
// Created by rock-trl on 11/14/19.
//

#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
	int fac = 1;
	for(int i = x; i > 0; i--)
		fac = fac * i;
	return fac;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
		const int d_order,                    // the order of derivative
		const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
		const Eigen::MatrixXd &Vel,           // boundary velocity
		const Eigen::MatrixXd &Acc,           // boundary acceleration
		const Eigen::VectorXd &Time)          // time allocation in each segment
{
	// enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
	int p_order   = 2 * d_order - 1;              // the order of polynomial
	int p_num1d   = p_order + 1;                  // the number of variables in each segment

	int m = Time.size();                          // the number of segments
	MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
	VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

	//A is different with Matlab
	/*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
	MatrixXd A = MatrixXd::Zero(m * p_num1d, m * p_num1d);
	for (auto k = 0; k < m; k++) {
		MatrixXd A_k = MatrixXd::Zero(p_num1d, p_num1d);
		for (auto i = 0; i < d_order; i++) {
			A_k.row(2 * i) = CalcTvec(p_num1d, i, 0.0);
			A_k.row(2 * i + 1) = CalcTvec(p_num1d, i, Time(k));
		}
		A.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = A_k;
	}
	MatrixXd A_inv = A.inverse();

	/*   Produce the dereivatives in X, Y and Z axis directly.  */
	VectorXd Dx = VectorXd::Zero(m * p_num1d);
	VectorXd Dy = VectorXd::Zero(m * p_num1d);
	VectorXd Dz = VectorXd::Zero(m * p_num1d);

	//D is different with Matlab
	for (auto k = 1; k < m + 1; k++) {
		Dx((k - 1) * p_num1d)     = Path((k - 1), 0);
		Dx((k - 1) * p_num1d + 1) = Path(k, 0);
		Dy((k - 1) * p_num1d)     = Path((k - 1), 1);
		Dy((k - 1) * p_num1d + 1) = Path(k, 1);
		Dz((k - 1) * p_num1d)     = Path((k - 1), 2);
		Dz((k - 1) * p_num1d + 1) = Path(k, 2);

		if (k == 1) {
			Dx((k - 1) * p_num1d + 2) = Vel(0, 0);
			Dy((k - 1) * p_num1d + 2) = Vel(0, 1);
			Dz((k - 1) * p_num1d + 2) = Vel(0, 2);

			Dx((k - 1) * p_num1d + 4) = Acc(0, 0);
			Dy((k - 1) * p_num1d + 4) = Acc(0, 1);
			Dz((k - 1) * p_num1d + 4) = Acc(0, 2);
		} else if (k == m) {
			Dx((k - 1) * p_num1d + 3) = Vel(1, 0);
			Dy((k - 1) * p_num1d + 3) = Vel(1, 1);
			Dz((k - 1) * p_num1d + 3) = Vel(1, 2);

			Dx((k - 1) * p_num1d + 5) = Acc(1, 0);
			Dy((k - 1) * p_num1d + 5) = Acc(1, 1);
			Dz((k - 1) * p_num1d + 5) = Acc(1, 2);
		}
	}

	/*   Produce the Minimum Snap cost function, the Hessian Matrix   */
	MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
	for (auto k = 0; k < m; k++) {
		for (auto i = d_order; i < p_num1d; i++) {
			for (auto j = i; j < p_num1d; j++) {
				auto a = i;
				auto b = j;
				auto c = i + j - p_order;
				for (auto idx = 1; idx < d_order; idx++) {
					a *= (a - idx);
					b *= (b - idx);
				}
				Q(k * p_num1d + i, k * p_num1d + j) = (double)a * (double)b * pow(Time(k), c) / (double)c;
				Q(k * p_num1d + j, k * p_num1d + i) = Q(k * p_num1d + i, k * p_num1d + j);
			}
		}
	}
	_Q = Q;

	int num_d, num_f, num_p;
	if (m > 1) {
		MatrixXd Ct;
		MatrixXd C;

		num_d = p_num1d * m;
		num_f = d_order + d_order + (m - 1) * (d_order - 1);
		num_p = (m - 1) * (d_order - 1);

		Ct = MatrixXd::Zero(num_d, num_f + num_p);

		// for the 1st segment
		{
			// enforcing the start states
			for (auto i = 0; i < d_order; i++)
				Ct(i * 2, i) = 1;

			// enforcing the position of the 2nd segment
			Ct(1, d_order) = 1;

			// setting other derivatives as free
			for (auto i = 1; i < d_order; i++)
				Ct(i * 2 + 1, num_f - 1 + i) = 1;
		}

		// for the last segment
		{
			// enforcing the final states
			for (auto i = 0; i < d_order; i++)
				Ct(p_num1d * (m - 1) + 2 * i + 1, num_f - 1 - (d_order - 1) + i) = 1;

			// enforcing the position of the 2nd to last segment
			Ct(p_num1d * (m - 1), num_f - 1 - (d_order - 1) - 1) = 1;

			// setting other derivatives as free
			for (auto i = 1; i < d_order; i++)
				Ct(p_num1d * (m - 1) + 2 * i, num_f + num_p - 1 - (d_order - 1) + i) = 1;
		}

		// for all meddle segments
		for(auto j = 1; j < m - 1; j ++ ) {
			// enforcing fixed position at the start and final at this particular segment
			Ct( p_num1d * j + 0, (d_order - 1) + 2 * j + 0 ) = 1;
			Ct( p_num1d * j + 1, (d_order - 1) + 2 * j + 1 ) = 1;

			// setting other derivatives as free
			for(auto i  = 1; i < d_order; i++) {
				Ct( p_num1d * j + 2 * i,     num_f + (d_order - 1) * (j - 1) + i - 1 ) = 1;
				Ct( p_num1d * j + 2 * i + 1, num_f + (d_order - 1) *  j      + i - 1 ) = 1;
			}
		}
		C = Ct.transpose();
		MatrixXd A_invC  = A_inv * Ct;

		VectorXd Dx1 = C * Dx;
		VectorXd Dy1 = C * Dy;
		VectorXd Dz1 = C * Dz;

		MatrixXd R   = A_invC.transpose() * _Q *  A_invC;

		VectorXd Dxf(num_f), Dyf(num_f), Dzf(num_f);
		Dxf = Dx1.segment(0, num_f);
		Dyf = Dy1.segment(0, num_f);
		Dzf = Dz1.segment(0, num_f);

		MatrixXd Rff(num_f, num_f);
		MatrixXd Rfp(num_f, num_p);
		MatrixXd Rpf(num_p, num_f);
		MatrixXd Rpp(num_p, num_p);

		Rff = R.block(0,     0,     num_f, num_f);
		Rfp = R.block(0,     num_f, num_f, num_p);
		Rpf = R.block(num_f, 0,     num_p, num_f);
		Rpp = R.block(num_f, num_f, num_p, num_p);

		MatrixXd Rpp_inv = Rpp.inverse();

		VectorXd Dxp(num_p), Dyp(num_p), Dzp(num_p);
		Dxp = - (Rpp_inv * Rfp.transpose()) * Dxf;
		Dyp = - (Rpp_inv * Rfp.transpose()) * Dyf;
		Dzp = - (Rpp_inv * Rfp.transpose()) * Dzf;

		Dx1.segment(num_f, num_p) = Dxp;
		Dy1.segment(num_f, num_p) = Dyp;
		Dz1.segment(num_f, num_p) = Dzp;

		Px = A_invC * Dx1;
		Py = A_invC * Dy1;
		Pz = A_invC * Dz1;
	} else {
		Px = A_inv * Dx;
		Py = A_inv * Dy;
		Pz = A_inv * Dz;
	}

	_Px = Px;
	_Py = Py;
	_Pz = Pz;

	for(auto i = 0; i < m; i ++) {
		PolyCoeff.block(i, 0 * p_num1d, 1, p_num1d) = Px.segment( i * p_num1d, p_num1d ).transpose();
		PolyCoeff.block(i, 1 * p_num1d, 1, p_num1d) = Py.segment( i * p_num1d, p_num1d ).transpose();
		PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = Pz.segment( i * p_num1d, p_num1d ).transpose();
	}

	return PolyCoeff;
}

//r = 0:pos 1:vel 2:acc 3:jerk
Eigen::VectorXd TrajectoryGeneratorWaypoint::CalcTvec(
		const int num,
		const int r,
		const double t)
{
	VectorXd tvec = VectorXd::Zero(num);
	for (auto i = r; i < num; i++) {
		tvec(i) = (double)Factorial(i) * pow(t, i - r) / (double)Factorial(i - r);
	}

	return tvec;
}
