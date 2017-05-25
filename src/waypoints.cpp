#include "waypoints.h"

#include <Eigen/QR>

using Eigen::ArrayXd;
using Eigen::Map;
using Eigen::MatrixXd;

Waypoints local(double px, double py, double ph, const Waypoints &global) {
  vector<double> lx;
  vector<double> ly;
  double cos_h = cos(ph);
  double sin_h = sin(ph);

  auto &gx = global[0];
  auto &gy = global[1];
  for (int i = 0, n = gx.size(); i < n; ++i) {
    double xt = gx[i] - px;
    double yt = gy[i] - py;

    double xi = xt * cos_h + yt * sin_h;
    double yi = yt * cos_h - xt * sin_h;

    lx.push_back(xi);
    ly.push_back(yi);
  }

  return {lx, ly};
}

Waypoints perform(double dt, double v0, const vector<double> &actuations) {
  double x = 0;
  double y = 0;
  double h = 0;
  double v = v0;

  Waypoints waypoints(2);
  auto a = actuations.begin();
  auto d = a + 1;

  for (auto n = actuations.end(); a != n; a += 2, d += 2) {
    v += *a * dt;
    h += *d * dt;
    x += cos(h) * v * dt;
    y += sin(h) * v * dt;
    waypoints[0].push_back(x);
    waypoints[1].push_back(y);
  }

  return waypoints;
}

VectorXd polyfit(const Waypoints &waypoints, int order) {
  int rows = waypoints[0].size();
  Map<const ArrayXd> xvals(waypoints[0].data(), rows);
  Map<const VectorXd> yvals(waypoints[1].data(), rows);

  MatrixXd A(rows, order + 1);
  A.block(0, 0, rows, 1).fill(1.0);
  for (int j = 0; j < order; j++) {
    auto Aj = A.block(0, j, rows, 1).array();
    A.block(0, j + 1, rows, 1) = Aj * xvals;
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
