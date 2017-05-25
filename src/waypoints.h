#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <Eigen/Dense>

#include <vector>

using namespace std;
using Eigen::VectorXd;

typedef vector<vector<double>> Waypoints;

/** @brief Speed contribution factor to the turning rate. */
const double Lf = 0.1;

/**
 * @brief Convert waypoints from global coordinate system to the given (local) system.
 */
Waypoints local(double px, double py, double ph, const Waypoints &global);

/**
 * @brief Compute waypoints from the given actuation vector.
 */
Waypoints perform(double dt, double v0, const vector<double> &actuations);

/**
 * @brief Fit waypoints to a polynomial of given order.
 */
VectorXd polyfit(const Waypoints &waypoints, int order);

#endif
