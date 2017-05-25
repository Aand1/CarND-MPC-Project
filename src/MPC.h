#ifndef MPC_H
#define MPC_H

#include "waypoints.h"

struct MPC {
  /** @brief Breadth of the time window, in time steps. */
  int breadth;

  /** @brief Length of the time step. */
  double dt;

  /** @brief Reference speed. */
  double vr;

  /**
   * @brief Create a new MPC controller over the given time window, step size and reference speed.
   */
  MPC(int breadth, double dt, double vr);

  /**
   * Class destructor.
   */
  virtual ~MPC();

  /**
   * @brief Compute a sequence of actuations to approach the given waypoints.
   */
  vector<double> operator () (double v0, const Waypoints &waypoints, int order = 2) const;
};

#endif /* MPC_H */
