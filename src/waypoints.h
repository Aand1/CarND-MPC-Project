#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <Eigen/Dense>

#include <vector>

using namespace std;
using Eigen::VectorXd;

typedef vector<vector<double>> Waypoints;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 0.05;

/**
 * @brief Convert waypoints from global coordinate system to the given (local) system.
 */
Waypoints local(double px, double py, double ph, const Waypoints &global);

/**
 * @brief Compute waypoints from the given actuation vector.
 */
Waypoints perform(double dt, double v0, double d0, const vector<double> &actuations);

/**
 * @brief Fit waypoints to a polynomial of given order.
 */
VectorXd polyfit(const Waypoints &waypoints, int order);

#endif
