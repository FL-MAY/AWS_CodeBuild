//
// Created by rock-trl on 11/14/19.
//

#ifndef PATH_COVERAGE_TRAJECTORY_GENERATOR_WAYPOINT_H
#define PATH_COVERAGE_TRAJECTORY_GENERATOR_WAYPOINT_H

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
private:
	double _qp_cost;
	Eigen::MatrixXd _Q;
	Eigen::VectorXd _Px, _Py, _Pz;
public:
	TrajectoryGeneratorWaypoint();

	~TrajectoryGeneratorWaypoint();

	Eigen::MatrixXd PolyQPGeneration(
			const int order,
			const Eigen::MatrixXd &Path,
			const Eigen::MatrixXd &Vel,
			const Eigen::MatrixXd &Acc,
			const Eigen::VectorXd &Time);

	int Factorial(int x);

	Eigen::VectorXd CalcTvec(
			const int num,
			const int r,
			const double t);
};


#endif //PATH_COVERAGE_TRAJECTORY_GENERATOR_WAYPOINT_H
