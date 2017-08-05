#include "road.h"

using namespace std;

void Road::update_road(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane){
  this->left_lane = left_lane;
  this->center_lane = center_lane;
  this->right_lane = right_lane;
}

vector<Vehicle> Road::get_lane_status(LANE lane){
  vector<Vehicle> r_lane;
  if (lane == LANE::LEFT) {
    r_lane = this->left_lane;
  } else if (lane == LANE::CENTER) {
    r_lane = this->center_lane;
  } else {
    r_lane = this->right_lane;
  }
  return r_lane;
}
