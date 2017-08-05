#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
#include "vehicle.h"
#include "utils.h"

using namespace std;

class Road {

  protected:
    vector<Vehicle> left_lane;
    vector<Vehicle> center_lane;
    vector<Vehicle> right_lane;

  public:
    Road() {};
    ~Road() {};

    void update_road(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane);
    vector<Vehicle> get_lane_status(LANE lane);
};

#endif // ROAD_H
