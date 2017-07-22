#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "vehicle.h"

using namespace std;

class Road {
  public:
    Road() {};
    ~Road() {};

    vector<Vehicle> left_lane;
    vector<Vehicle> center_lane;
    vector<Vehicle> right_lane;

};

#endif // ROAD_H
