#define HAVE_CSTDDEF

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

static const int A = 0;
static const int D = 1;
static const int SIZEOF_ACTUATION = 2;

struct Cost {
  /** @brief Basic scalar value. */
  typedef AD<double> Scalar;

  /** @brief Differentiable variable vector type. */
  typedef CPPAD_TESTVECTOR(Scalar) ADvector;

  /** @brief Breadth of the time window, in time steps. */
  int breadth;

  /** @brief Size of the time step. */
  Scalar dt;

  /** @brief Initial speed at the beginning of the time window. */
  Scalar v0;

  /** @brief Reference speed. */
  Scalar vr;

  /** @brief Initial turning rate. */
  Scalar d0;

  /** @brief Coefficients of the polynomial describing the reference route. */
  VectorXd route;

  /**
   * @brief Create a new optimization task with given initial speed and reference route.
   */
  Cost(int breadth, double dt, double v0, double vr, double d0, const VectorXd &route) {
    this->breadth = breadth;
    this->dt = dt;
    this->v0 = v0;
    this->vr = vr;
    this->d0 = d0;
    this->route = route;
  }

  void operator () (ADvector &fg, const ADvector &vars) {
    Scalar vt = v0;
    Scalar ht = 0; //vt * Lf * d0 * dt;
    Scalar xt = 0; //CppAD::cos(ht) * vt * dt;
    Scalar yt = 0; //CppAD::sin(ht) * vt * dt;

    for (int i = 0; i < breadth; ++i) {
      // Compute the controller-proposed state at time (i * dt).
      auto &a = vars[A + SIZEOF_ACTUATION * i];
      auto &d = vars[D + SIZEOF_ACTUATION * i];
      ht += vt * Lf * d * dt;
      vt += a * dt;
      xt += CppAD::cos(ht) * vt * dt;
      yt += CppAD::sin(ht) * vt * dt;

      // Compute the reference state at time (i * dt).
      auto yr = reference(xt);
      auto hr = CppAD::atan2(yr, xt);

      // Compute the contribution at time i * dt to the cost function.
      fg[0] += CppAD::pow(yt - yr, 2);
      fg[0] += CppAD::pow(ht - hr, 2);
      fg[0] += CppAD::pow(vt - vr, 2);

      // Minimize actuator use.
      fg[0] += CppAD::pow(a, 2);
      fg[0] += CppAD::pow(d, 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 1; i < breadth; ++i) {
      auto &a1 = vars[A + SIZEOF_ACTUATION * (i - 1)];
      auto &d1 = vars[D + SIZEOF_ACTUATION * (i - 1)];

      auto &a2 = vars[A + SIZEOF_ACTUATION * i];
      auto &d2 = vars[D + SIZEOF_ACTUATION * i];

      fg[0] += CppAD::pow(a1 - a2, 2);
      fg[0] += 10 * CppAD::pow(d1 - d2, 2);
    }
  }

private:
  Scalar reference(const Scalar &x) const {
    Scalar y = route(0);
    for (int i = 1, n = route.rows(); i < n; ++i) {
      y += route(i) * CppAD::pow(x, i);
    }

    return y;
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(int breadth, double dt, double vr) {
    this->breadth = breadth;
    this->dt = dt;
    this->vr = vr;
}

MPC::~MPC() {
  // Nothing to do.
}

vector<double> MPC::operator () (double v0, double d0, const Waypoints &waypoints, int order) const {
  // Differentiable value vector type.
  typedef CPPAD_TESTVECTOR(double) Vector;

  // Independent variables and bounds.
  Vector vars(breadth * SIZEOF_ACTUATION);
  Vector vars_lowerbound(breadth * SIZEOF_ACTUATION);
  Vector vars_upperbound(breadth * SIZEOF_ACTUATION);

  // Constraint bounds, set to size 0 as the cost function includes no constraints.
  Vector constraints_lowerbound(0);
  Vector constraints_upperbound(0);

  // Initialize independent variable and bounds vectors.
  for (int i = 0; i < breadth; i++) {
    int i_a = A + i * SIZEOF_ACTUATION;
    int i_d = D + i * SIZEOF_ACTUATION;

    vars[i_a] = 0;
    vars[i_d] = 0;

    vars_lowerbound[i_a] = -1.0;
    vars_upperbound[i_a] = 1.0;

    vars_lowerbound[i_d] = -0.436332;
    vars_upperbound[i_d] = 0.436332;
  }

  // Fit a polynomial to the waypoints.
  VectorXd route = polyfit(waypoints, order);

  // Define the cost function.
  Cost cost(breadth, dt, v0, vr, d0, route);

  // Options for IPOPT solver.
  std::string options =
    "Integer print_level  0\n"
    "Sparse  true forward\n"
    "Sparse  true reverse\n"
    "Numeric max_cpu_time 0.5\n";

  // Solution to the cost optimization problem.
  CppAD::ipopt::solve_result<Vector> solution;

  // Call the solver on the cost function and given parameters.
  CppAD::ipopt::solve<Vector, Cost>(
    options,
    vars,
    vars_lowerbound,
    vars_upperbound,
    constraints_lowerbound,
    constraints_upperbound,
    cost,
    solution
  );

  // Report solution results.
  auto value = solution.obj_value;
  auto status = (solution.status == CppAD::ipopt::solve_result<Vector>::success ? "succeeded" : "failed");
  std::cout << "Solver " << status << ", final cost value = " << value << std::endl;

  vector<double> actuations;
  auto &x = solution.x;
  for (int i = 0, n = breadth * SIZEOF_ACTUATION; i < n; ++i) {
    actuations.push_back(x[i]);
  }

  return actuations;
}
