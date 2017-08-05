#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
#include <cmath>
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

    bool safe_lane(Vehicle& car, LANE lane);
    bool free_lane(Vehicle& car, LANE lane);
    LANE lane_change_available(Vehicle& car);
};

#endif // ROAD_H
