#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "road.h"

using namespace std;

class Planner {
public:
  Planner(){};
  ~Planner(){};

  vector<double> JMT(vector<double> start, vector <double> end, double T);

};

#endif /* PLANNER_H */
