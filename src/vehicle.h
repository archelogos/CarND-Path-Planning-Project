#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "utils.h"

using namespace std;

class Vehicle {

  /* ["sensor_fusion"] A 2d vector of cars and then that car's [
  id: car's unique ID,
  x: car's x position in map coordinates,
  y: car's y position in map coordinates,
  v: car's speed,
  s: car's s position in frenet coordinates,
  d: car's d position in frenet coordinates. */

  protected:

    int id;
    double x;
    double y;
    double v;
    double s;
    double d;
    double yaw;
    bool stopped;
    STATE state;

    vector<double> previous_s;
    vector<double> previous_d;

  public:

    Vehicle();
    Vehicle(int id, double x, double y, double v, double s, double d);
    ~Vehicle(){};

    int get_id();
    double get_x();
    double get_y();
    double get_v();
    double get_s();
    double get_d();
    double get_yaw();
    STATE get_state();
    LANE lane();

    void update_vehicle_state(double x, double y, double v, double s, double d, double yaw);
    bool is_stopped();
    void lock_start();
    void set_previous_s(vector<double> previous_s);
    void set_previous_d(vector<double> previous_d);
    vector<double> prev_s();
    vector<double> prev_d();
    void set_state(STATE state);

};

#endif // VEHICLE_H
