#include "planner.h"

using namespace std;

/* JMT*/
vector<double> Planner::JMT(vector<double> start, vector <double> end, double T) {
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
  Eigen::MatrixXd A(3,3);
  Eigen::MatrixXd B(3,1);

  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;

  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai*B;

  return {start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2]};
}

void Planner::estimate_new_points(Map& map, vector<vector<double>>& new_points){

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // jmt
  double T = this->n * AT;
  vector<double> poly = this->JMT(this->start_s, end_s, T);

  double t, next_s, mod_s;
  vector <double> XY;
  for(int i = 0; i < n; i++) {

    /* JMT */
    cout << "----------JMT----------" << endl;
    t = AT*i;
    cout << "t= " << t << endl;
    next_s = 0.0;
    for (int a = 0; a < poly.size(); ++a) {
      next_s += poly[a] * pow(t, a);
    }
    mod_s = fmod(next_s, TRACK_DISTANCE);

    XY = map.getXY(mod_s);

    next_x_vals.push_back(XY[0]);
    next_y_vals.push_back(XY[1]);
  }

  new_points = {next_x_vals, next_y_vals};

}

void Planner::create_trajectory(Map& map, Vehicle& car, vector<vector<double>>& new_points) {

  // first trajectory
  if (car.is_stopped()) {

    // block
    car.lock_start();

    this->n = 50*4.5;
    double target_s = car.get_s() + 40.0;
    double target_v = SPEED_LIMIT;

    this->start_s = {car.get_s(), car.get_v(), 0.0};
    this->end_s= {target_s, target_v, 0.0};

  } else {

    this->n = CYCLES*POINTS;
    double target_s = car.prev_s()[0] + AT*CYCLES*POINTS * car.prev_s()[1];
    double target_v = car.prev_s()[1];

    this->start_s = {car.prev_s()[0], car.prev_s()[1], 0.0};
    this->end_s = {target_s, target_v, 0.0};

  }

  car.set_previous_s(end_s);
  this->estimate_new_points(map, new_points);
}
