#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <cmath>
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathPlanner {
public:
  PathPlanner();
  virtual ~PathPlanner();

  double s_inc;
  double d;
  double target_s_inc;
  double target_d;

  vector<double> JMT(vector<double> start, vector <double> end, double T);

};

/* IMPLEMENTATION */

PathPlanner::PathPlanner() {
  this->s_inc = 0.42;
  this->target_s_inc = 0.42;
  this->d = 6.0;
  this->target_d = 6.0;
}
PathPlanner::~PathPlanner() {}


vector<double> PathPlanner::JMT(vector<double> start, vector <double> end, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  */

  // prepare matrix A with coefficents
  int w = 3;
  int h = 3;
  MatrixXd A(w,h);

  A(0,0) = T*T*T;
  A(0,1) = T*T*T*T;
  A(0,2) = T*T*T*T*T;
  A(1,0) = 3*T*T;
  A(1,1) = 4*T*T*T;
  A(1,2) = 5*T*T*T*T;
  A(2,0) = 6*T;
  A(2,1) = 12*T*T;
  A(2,2) = 20*T*T*T;

  // prepare coefficents for vector
  double si         = start[0];
  double si_dot     = start[1];
  double si_dot_dot = start[2];
  double sf         = end[0];
  double sf_dot     = end[1];
  double sf_dot_dot = end[2];

  VectorXd b(3);
  b(0) = sf - (si + si_dot*T + 0.5*si_dot_dot*T*T);
  b(1) = sf_dot - (si_dot + si_dot_dot*T);
  b(2) = sf_dot_dot - si_dot_dot;

  // solve the systemof lenar equations using eigen library
  VectorXd x = A.colPivHouseholderQr().solve(b);

  // compose solution
  double a0 = si;
  double a1 = si_dot;
  double a2 = si_dot_dot / 2.0;
  double a3 = x[0];
  double a4 = x[1];
  double a5 = x[2];

  return {a0,a1,a2,a3,a4,a5};
}

#endif /* PATHPLANNER_H */
