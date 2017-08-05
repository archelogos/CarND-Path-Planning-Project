#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "map.h"
#include "road.h"

using namespace std;

class Planner {

protected:

  int n;
  vector<double> start_s;
  vector<double> end_s;
  vector<double> start_d;
  vector<double> end_d;

public:
  Planner(){};
  ~Planner(){};

  vector<double> JMT(vector<double> start, vector <double> end, double T);
  void estimate_new_points(Map& map, vector<vector<double>>& new_points);
  void create_trajectory(Map& map, Vehicle& car, vector<vector<double>>& new_points);

};

#endif /* PLANNER_H */
